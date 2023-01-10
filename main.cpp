#include <iostream>
#include <cassert>
#include <cmath>
#include <netinet/in.h>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PORT 8080
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

const float_t NB_VALUES = 40000.;
const float_t g = 9.80926; // m/s^2
const float_t p = 48.41846 / 180.0 * M_PI; // rad initial latitude
const float_t w = 7.2921150e-5; // rad/s
const float_t df = 200.; // Hz

Eigen::Matrix3d W;

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;

#pragma pack(push, 1)
struct Binary_Data{
  U8 syncro; // 0x55
  U8 counter; // Binary
  U16 status; // Binary MSB
  U32 data[6]; // Binary MSB
  U8 temperature; //  Binary
  U16 checksum; // Binary LSB
  U8 stopByte; // 0xAA
};
#pragma pack(pop)

struct Inertial_Data{
  float_t data[6];

  void set(Binary_Data& binDat){
    assert(binDat.syncro == 0x55);
    assert(binDat.stopByte == 0xAA);
    for (int i = 0; i < 6; i++){
      binDat.data[i] = ntohl(binDat.data[i]);
    }
    memcpy(data, binDat.data, sizeof(binDat.data));
  }
};

struct Network_Params{
  int server_fd;
  struct sockaddr_in cliAddr;
  socklen_t cliAddrLen;
};

class imu_pub : public rclcpp::Node{
  public:
    sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    imu_pub() : Node("imu")
    {
      pub = this->create_publisher<sensor_msgs::msg::Imu>("imu", 200);
      imu_msg.header.frame_id = "map";
    }
};

void printProgress(double percentage){
  /** \brief Shows a progress bar in the shell */
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

int connect(Network_Params& netwparams){
  netwparams.server_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (netwparams.server_fd < 0){
    std::cerr << "Erreur lors de la création du socket" << std::endl;
    return EXIT_FAILURE;
  }

  // Configurer l'adresse du socket serveur
  struct sockaddr_in addr;
  netwparams.cliAddrLen = sizeof(netwparams.cliAddr);
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(PORT);

  // Lier l'adresse au socket serveur
  if (bind(netwparams.server_fd, (struct sockaddr*) &addr, sizeof(addr)) < 0){
    std::cerr << "Erreur lors de la liaison de l'adresse au socket" << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int load_data(Network_Params& netwparams, Binary_Data& binDat, Inertial_Data& inDat){
  char buff[32] = {0};
  ssize_t bytes_received = recvfrom(netwparams.server_fd, buff, sizeof(buff), 0, (struct sockaddr*)&netwparams.cliAddr, &netwparams.cliAddrLen);
  if (bytes_received < 0){
    std::cerr << "Erreur lors de la lecture des données entrantes" << std::endl;
    return EXIT_FAILURE;
  }
  memcpy(&binDat, &buff, sizeof(buff));
  inDat.set(binDat);
  return EXIT_SUCCESS;
}

void publish_ros(imu_pub& node, const tf2::Quaternion& q_att, const Eigen::Vector3d& delta_a, const Eigen::Vector3d& delta_v){

  node.imu_msg.header.stamp = node.get_clock()->now();
  node.imu_msg.orientation = tf2::toMsg(q_att);
  node.imu_msg.angular_velocity.x = delta_a(0);
  node.imu_msg.angular_velocity.y = delta_a(1);
  node.imu_msg.angular_velocity.z = delta_a(2);
  node.imu_msg.linear_acceleration.x = delta_v(0);
  node.imu_msg.linear_acceleration.y = delta_v(1);
  node.imu_msg.linear_acceleration.z = delta_v(2);
  node.pub->publish(node.imu_msg);

}

void rotation_integration_iC(Eigen::Matrix3d& C_b_i, Eigen::Vector3d& nu_ib_i, const Eigen::Vector3d& alpha_ib_b, const Eigen::Vector3d& nu_ib_b){
  // Calculer la matrice A en utilisant les composantes de l'accélération angulaire alpha_ib_b
  Eigen::Matrix3d A;
  A << 1., -alpha_ib_b(2), alpha_ib_b(1),
        alpha_ib_b(2), 1., -alpha_ib_b(0),
        -alpha_ib_b(1), alpha_ib_b(0), 1.;

  // Mettre à jour la matrice de rotation C_b_i en multipliant par la matrice A
  C_b_i = C_b_i * A; // DL ordre 1

  // Calculer la vitesse nu_ib_i en multipliant la matrice de rotation C_b_i par la vitesse nu_ib_b
  nu_ib_i = C_b_i * nu_ib_b;

  // Orthonormaliser la matrice de rotation
  Eigen::Quaternion<double> q(C_b_i);
  C_b_i = q.normalized().toRotationMatrix();
}

void compute_gn(std::vector<Eigen::Vector3d>& g_n0){
  // Calcul du vecteur g_n0 avec un modèle de gravité simplifié
  for (size_t i = 0; i < NB_VALUES; i++){
    Eigen::Vector3d g_i = {g * cos(p) * sin(p) * (cos(w * i / df) - 1), -g * cos(p) * sin(w * i / df), g * (pow(cos(p), 2) * cos(w * i / df) + pow(sin(p), 2))};
    g_n0.push_back(g_i);
  }
}

void compute_gb(Network_Params& netwparams, std::vector<Eigen::Vector3d>& g_b0, Eigen::Matrix3d& C_b_i, Binary_Data& binDat, Inertial_Data& inDat){

  size_t c = 0;
  Eigen::Vector3d g_b_i;

  while (c < NB_VALUES){

    printProgress(c / NB_VALUES);
    c++;

    load_data(netwparams, binDat, inDat);

    Eigen::Vector3d delta_a(inDat.data[0], inDat.data[1], inDat.data[2]);

    Eigen::Vector3d delta_v(inDat.data[3], inDat.data[4], inDat.data[5]);

    // Calcul du vecteur gravité g_b0 dans le repère initial
    rotation_integration_iC(C_b_i, g_b_i, delta_a, delta_v);
    g_b0.push_back(-g_b_i * df);

  }
}

void align_inertial_sensor(const std::vector<Eigen::Vector3d>& g_n0, const std::vector<Eigen::Vector3d>& g_b0, Eigen::Vector3d& att){
  Eigen::Matrix3d C = Eigen::Matrix3d::Identity();

  for (size_t i = 0; i < g_n0.size(); i++){
    // Compute the gravity vector in the initial frame
    Eigen::Vector3d g0;
    g0 << g_n0[i][0], g_n0[i][1], g_n0[i][2];

    // Compute the gravity vector in the current frame
    Eigen::Vector3d gb;
    gb << g_b0[i][0], g_b0[i][1], g_b0[i][2];

    // Compute the rotation matrix using the Kabsch algorithm
    C += g0 * gb.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    // Handle the case where the determinant of R is negative
    if (R.determinant() < 0){
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      I(2, 2) = -1;
      R = svd.matrixU() * I * svd.matrixV().transpose();
    }

    // Orthonormaliser la matrice de rotation
    Eigen::Quaternion<double> q(R);
    R = q.normalized().toRotationMatrix();

    // Calculer les angles à partir de la matrice de rotation
    att << atan2(R(1,0), R(0,0)),
            atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))),
            atan2(R(2,1), R(2,2));

  }
}

void initialize_inertial_sensor(Network_Params& netwparams, Eigen::Matrix3d& C_b_i, Eigen::Vector3d& att){

  Binary_Data binDat;
  Inertial_Data inDat;

  std::vector<Eigen::Vector3d> g_b0;
  std::vector<Eigen::Vector3d> g_n0;

  std::cout << "Alignement :" << '\n';

  compute_gn(g_n0);
  compute_gb(netwparams, g_b0, C_b_i, binDat, inDat);

  // Calcul de l'attitude initiale
  align_inertial_sensor(g_n0, g_b0, att);

  std::cout << "Alignement terminé." << '\n';

}

void mechanization(Network_Params& netwparams, Eigen::Matrix3d& C_b_i, Eigen::Vector3d& att0, Eigen::Vector3d& delta_a, Eigen::Vector3d& delta_v, tf2::Quaternion& q_att){

  Binary_Data binDat;
  Inertial_Data inDat;
  Eigen::Vector3d att;

  load_data(netwparams, binDat, inDat);
  delta_a << inDat.data[0], inDat.data[1], inDat.data[2];
  delta_v << inDat.data[3], inDat.data[4], inDat.data[5];

  Eigen::Matrix3d A; // DL ordre 1
  A << 1., -delta_a(2), delta_a(1),
        delta_a(2), 1., -delta_a(0),
        -delta_a(1), delta_a(0), 1.;
  C_b_i = C_b_i * A - w / df * W * C_b_i;

  // Orthonormaliser la matrice de rotation
  Eigen::Quaternion<double> q(C_b_i);
  C_b_i = q.normalized().toRotationMatrix();

  // att est la variation d'angle par rapport à l'alignement compté dans le sens trigonométrique
  att << atan2(C_b_i(1,0), C_b_i(0,0)),
          atan2(-C_b_i(2,0), sqrt(C_b_i(2,1)*C_b_i(2,1) + C_b_i(2,2)*C_b_i(2,2))),
          atan2(C_b_i(2,1), C_b_i(2,2));
  att = att0 - att; // On soustrait car la boussole est dans le sens anti-trigonométrique
  q_att.setRPY(att(0), att(1), att(2));
}

int main(int argc, char* argv[]){

  // Approximation de lattitude constante
  W << 0., sin(p), 0.,
  -sin(p), 0, -cos(p),
  0., cos(p), 0.;

  Network_Params netwparams;
  connect(netwparams);

  // Matrice de rotation C_b_i et attitude initiale
  Eigen::Matrix3d C_b_i = Eigen::Matrix3d::Identity();
  Eigen::Vector3d att0;

  initialize_inertial_sensor(netwparams, C_b_i, att0);

  Eigen::Vector3d delta_a;
  Eigen::Vector3d delta_v;
  tf2::Quaternion q_att;

  rclcpp::init(argc, argv);
  imu_pub node;
  std::cout << "Suscribe to ROS2 topic : \"imu\"" << '\n';

  while (rclcpp::ok()){

    mechanization(netwparams, C_b_i, att0, delta_a, delta_v, q_att);

    publish_ros(node, q_att, delta_a, delta_v);

  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;

}
