#include <hl_communication/utils.h>

#include <opencv2/calib3d.hpp>

#include <google/protobuf/util/message_differencer.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

using google::protobuf::util::MessageDifferencer;

using namespace std::chrono;

namespace hl_communication
{
bool operator<(const RobotIdentifier& id1, const RobotIdentifier& id2)
{
  return (id1.team_id() == id2.team_id() && id1.robot_id() < id2.robot_id()) || id1.team_id() < id2.team_id();
}

bool operator==(const RobotIdentifier& id1, const RobotIdentifier& id2)
{
  return id1.team_id() == id2.team_id() && id1.robot_id() == id2.robot_id();
}

bool operator<(const MsgIdentifier& id1, const MsgIdentifier& id2)
{
  if (!id1.has_src_ip() || !id2.has_src_ip() || !id1.has_src_port() || !id2.has_src_port())
  {
    throw std::runtime_error("Incomplete message identifier");
  }
  if (id1.src_ip() != id2.src_ip())
  {
    return id1.src_ip() < id2.src_ip();
  }
  if (id1.src_port() != id2.src_port())
  {
    return id1.src_port() < id2.src_port();
  }
  return id1.packet_no() < id2.packet_no();
}

bool operator<(const RobotCameraIdentifier& id1, const RobotCameraIdentifier& id2)
{
  if (MessageDifferencer::Equals(id1.robot_id(), id2.robot_id()))
    return id1.camera_name() < id2.camera_name();
  return id1.robot_id() < id2.robot_id();
}

bool operator<(const VideoSourceID& id1, const VideoSourceID& id2)
{
  bool start_before = id1.has_utc_start() && id2.has_utc_start() && id1.utc_start() < id2.utc_start();
  if (id1.has_robot_source())
  {
    if (id2.has_robot_source())
    {
      if (id1.robot_source() < id2.robot_source())
        return true;
      else if (id2.robot_source() < id1.robot_source())
        return false;
      else
        return start_before;
    }
    else
    {
      return true;
    }
  }
  else if (id1.has_external_source())
  {
    if (id2.has_external_source())
    {
      if (id1.external_source() < id2.external_source())
        return true;
      else if (id2.external_source() < id1.external_source())
        return false;
      else
        return start_before;
    }
    else
    {
      return false;
    }
  }
  throw std::logic_error(HL_DEBUG + "Incomplete videoSourceId");
}

void readFromFile(const std::string& path, google::protobuf::Message* msg)
{
  msg->Clear();
  std::ifstream in(path);
  if (!in.good())
  {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  msg->ParseFromIstream(&in);
}

void writeToFile(const std::string& path, const google::protobuf::Message& msg)
{
  std::ofstream out(path, std::ios::binary);
  if (!out.good())
  {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  msg.SerializeToOstream(&out);
}

uint64_t getTimeStamp()
{
  return duration_cast<duration<uint64_t, std::micro>>(steady_clock::now().time_since_epoch()).count();
}

uint64_t getUTCTimeStamp()
{
  return duration_cast<duration<uint64_t, std::micro>>(system_clock::now().time_since_epoch()).count();
}

int64_t getSteadyClockOffset()
{
  int64_t steady_ts = getTimeStamp();
  int64_t system_ts = getUTCTimeStamp();
  return system_ts - steady_ts;
}

std::string getPrettyDuration(uint64_t duration_us)
{
  std::ostringstream oss;
  uint64_t millisecond_duration = 1000;
  uint64_t sec_duration = 1000 * millisecond_duration;
  uint64_t min_duration = 60 * sec_duration;
  uint64_t hour_duration = 60 * min_duration;
  uint64_t day_duration = 24 * hour_duration;
  if (duration_us >= day_duration)
  {
    int nb_days = duration_us / day_duration;
    duration_us -= nb_days * day_duration;
    oss << nb_days << "d";
  }
  if (duration_us >= hour_duration)
  {
    int nb_hours = duration_us / hour_duration;
    duration_us -= nb_hours * hour_duration;
    oss << nb_hours << "h";
  }
  if (duration_us >= min_duration)
  {
    int nb_mins = duration_us / min_duration;
    duration_us -= nb_mins * min_duration;
    oss << nb_mins << "m";
  }
  if (duration_us >= sec_duration)
  {
    int nb_secs = duration_us / sec_duration;
    duration_us -= nb_secs * sec_duration;
    oss << nb_secs << "s";
  }
  int nb_milliseconds = duration_us / millisecond_duration;
  duration_us -= nb_milliseconds * millisecond_duration;
  oss << nb_milliseconds << "ms";
  return oss.str();
}

std::string getBaseName(const std::string& path)
{
  size_t idx = path.find_last_of('/');
  if (idx == std::string::npos)
  {
    return path;
  }
  return path.substr(idx + 1);
}

uint64_t stringToIP(const std::string& str)
{
  std::stringstream ss(str);
  uint64_t result = 0;
  std::string element;
  while (getline(ss, element, '.'))
  {
    uint64_t elem_value = std::stoi(element);
    result = (result << 8) + elem_value;
  }
  return result;
}

std::string ipToString(uint64_t ip)
{
  std::ostringstream oss;
  oss << (ip >> 24 & 0xFF) << "." << (ip >> 16 & 0xFF) << "." << (ip >> 8 & 0xFF) << "." << (ip & 0xFF);
  return oss.str();
}

void invertPosition(PositionDistribution* position)
{
  position->set_x(-position->x());
  position->set_y(-position->y());
}

void invertAngle(AngleDistribution* angle)
{
  double alpha = angle->mean() + M_PI;
  if (alpha > M_PI)
  {
    alpha -= 2 * M_PI;
  }
  angle->set_mean(alpha);
}

void invertPose(PoseDistribution* pose)
{
  if (pose->has_position())
  {
    invertPosition(pose->mutable_position());
  }
  if (pose->has_dir())
  {
    invertAngle(pose->mutable_dir());
  }
}

bool exportUncertainty(const PositionDistribution& p, cv::Mat* out)
{
  int nb_coeffs = p.uncertainty_size();
  if (nb_coeffs != 2 && nb_coeffs != 3)
    return false;
  double var_x(0), var_y(0), var_xy(0);
  var_x = p.uncertainty(0);
  if (nb_coeffs == 3)
  {
    var_xy = p.uncertainty(1);
    var_y = p.uncertainty(2);
  }
  else
  {
    var_y = p.uncertainty(1);
  }
  *out = cv::Mat(2, 2, CV_32F);
  out->at<float>(0, 0) = var_x;
  out->at<float>(0, 1) = var_xy;
  out->at<float>(1, 0) = var_xy;
  out->at<float>(1, 1) = var_y;
  return true;
}

bool isPenalized(const GCMsg& msg, int team_id, int robot_id)
{
  for (const GCTeamMsg& team : msg.teams())
  {
    if (!team.has_team_number() || team.team_number() != team_id)
    {
      continue;
    }
    int idx = robot_id - 1;  // Robots are numbered from 1
    return team.robots(idx).has_penalty() && team.robots(idx).penalty() != 0;
  }
  return false;
}

std::string getFormattedTime()
{
  system_clock::time_point now = system_clock::now();
  time_t tt = system_clock::to_time_t(now);
  struct tm tm;
  localtime_r(&tt, &tm);
  char buffer[80];  // Buffer is big enough
  sprintf(buffer, "%04d_%02d_%02d_%02dh%02dm%02ds", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
          tm.tm_sec);
  return std::string(buffer);
}

void intrinsicToCV(const IntrinsicParameters& camera_parameters, cv::Mat* camera_matrix,
                   cv::Mat* distortion_coefficients, cv::Size* img_size)
{
  *camera_matrix = cv::Mat(3, 3, CV_64F);
  camera_matrix->at<double>(0, 0) = camera_parameters.focal_x();
  camera_matrix->at<double>(1, 1) = camera_parameters.focal_y();
  camera_matrix->at<double>(0, 2) = camera_parameters.center_x();
  camera_matrix->at<double>(1, 2) = camera_parameters.center_y();
  camera_matrix->at<double>(2, 2) = 1.0;
  img_size->width = camera_parameters.img_width();
  img_size->height = camera_parameters.img_height();
  *distortion_coefficients = cv::Mat(1, camera_parameters.distortion_size(), CV_64F);
  for (int i = 0; i < camera_parameters.distortion_size(); i++)
  {
    distortion_coefficients->at<double>(0, i) = camera_parameters.distortion(i);
  }
}

void cvToIntrinsic(const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients, const cv::Size& img_size,
                   IntrinsicParameters* camera_parameters)
{
  camera_parameters->Clear();
  camera_parameters->set_focal_x(camera_matrix.at<double>(0, 0));
  camera_parameters->set_focal_y(camera_matrix.at<double>(1, 1));
  camera_parameters->set_center_x(camera_matrix.at<double>(0, 2));
  camera_parameters->set_center_y(camera_matrix.at<double>(1, 2));
  camera_parameters->set_img_width(img_size.width);
  camera_parameters->set_img_height(img_size.height);
  for (int i = 0; i < distortion_coefficients.cols; i++)
  {
    camera_parameters->add_distortion(distortion_coefficients.at<double>(0, i));
  }
}

cv::Mat rodriguesFromQuaternion(double qw, double qx, double qy, double qz)
{
  // From: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
  cv::Mat res(3, 1, CV_64F);
  double angle = 2 * acos(qw);
  double s = std::sqrt(1 - qw * qw);
  if (s > std::pow(10, -9))
  {
    qx /= s;
    qy /= s;
    qz /= s;
  }
  res.at<double>(0, 0) = qx * angle;
  res.at<double>(1, 0) = qy * angle;
  res.at<double>(2, 0) = qz * angle;
  return res;
}

void pose3DToCV(const Pose3D& pose, cv::Mat* rvec, cv::Mat* tvec)
{
  // Read rotation
  if (pose.rotation_size() == 3)
  {
    *rvec = cv::Mat(3, 1, CV_64F);
    for (int i = 0; i < 3; i++)
    {
      rvec->at<double>(i, 0) = pose.rotation(i);
    }
  }
  else if (pose.rotation_size() == 4)
  {
    *rvec = rodriguesFromQuaternion(pose.rotation(0), pose.rotation(1), pose.rotation(2), pose.rotation(3));
  }
  else
  {
    throw std::runtime_error("Only Rodrigues rotation vector and quaternions are supported currently");
  }
  // Read translation
  if (pose.translation_size() == 3)
  {
    *tvec = cv::Mat(3, 1, CV_64F);
    for (int i = 0; i < 3; i++)
    {
      tvec->at<double>(i, 0) = pose.translation(i);
    }
  }
  else
  {
    throw std::runtime_error("Size of translation in Pose3D is not valid (only 3 is accepted)");
  }
}

void cvToPose3D(const cv::Mat& rvec, const cv::Mat& tvec, Pose3D* pose)
{
  pose->Clear();
  for (int i = 0; i < 3; i++)
  {
    pose->add_rotation(rvec.at<double>(i, 0));
    pose->add_translation(tvec.at<double>(i, 0));
  }
}

Eigen::Affine3d getAffineFromProtobuf(const Pose3D& pose)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d src_in_dst = Eigen::Vector3d::Zero();
  switch (pose.rotation_size())
  {
    case 0:
      break;
    case 3: {
      // Rodrigues vector case
      Eigen::Vector3d rotation_axis(pose.rotation(0), pose.rotation(1), pose.rotation(2));
      double angle = 0;
      if (rotation_axis.norm() > std::pow(10, -6))
      {
        angle = rotation_axis.norm();
        rotation_axis.normalize();
      }
      else
      {
        rotation_axis = Eigen::Vector3d::UnitZ();
      }
      rotation = Eigen::AngleAxisd(angle, rotation_axis);
      break;
    }
    case 4: {
      Eigen::Quaterniond q(pose.rotation(0), pose.rotation(1), pose.rotation(2), pose.rotation(3));
      rotation = Eigen::Matrix3d(q);
      break;
    }
    default:
      throw std::runtime_error(HL_DEBUG + " invalid size for rotation: " + std::to_string(pose.rotation_size()));
  }
  switch (pose.translation_size())
  {
    case 0:
      break;
    case 3:
      for (int dim = 0; dim < 3; dim++)
      {
        src_in_dst(dim) = pose.translation(dim);
      }
      break;
    default:
      throw std::runtime_error(HL_DEBUG + " invalid size for src_in_dst: " + std::to_string(pose.translation_size()));
  }
  return Eigen::Translation3d(src_in_dst) * Eigen::Affine3d(rotation);
}

void setProtobufFromAffine(const Eigen::Affine3d& affine, Pose3D* pose)
{
  Eigen::Vector3d src_in_dst = affine * Eigen::Vector3d::Zero();
  pose->clear_rotation();
  pose->clear_translation();
  Eigen::Quaterniond q(affine.linear());
  pose->add_rotation(q.w());
  pose->add_rotation(q.x());
  pose->add_rotation(q.y());
  pose->add_rotation(q.z());
  pose->add_translation(src_in_dst(0));
  pose->add_translation(src_in_dst(1));
  pose->add_translation(src_in_dst(2));
}

std::ostream& operator<<(std::ostream& out, const Pose3D& pose)
{
  cv::Mat rvec, tvec;
  pose3DToCV(pose, &rvec, &tvec);
  return out << "tvec: " << tvec << " rvec: " << rvec;
}

cv::Point3f cvtToPoint3f(const PositionDistribution& position)
{
  return cv::Point3f(position.x(), position.y(), 0);
}

cv::Point3f fieldFromSelf(const PositionDistribution& obj_pos_in_self, const PoseDistribution& robot_pose)
{
  cv::Point3f pos_in_self = cvtToPoint3f(obj_pos_in_self);
  cv::Point3f robot_pos = cvtToPoint3f(robot_pose.position());
  double robot_dir = robot_pose.dir().mean();
  cv::Point3f offset;
  offset.x = pos_in_self.x * cos(robot_dir) - pos_in_self.y * sin(robot_dir);
  offset.y = pos_in_self.x * sin(robot_dir) + pos_in_self.y * cos(robot_dir);
  return robot_pos + offset;
}

cv::Size getImgSize(const CameraMetaInformation& camera_information)
{
  const IntrinsicParameters& p = camera_information.camera_parameters();
  return cv::Size(p.img_width(), p.img_height());
}

cv::Point3f fieldToCamera(const cv::Point3f& pos_in_field, const cv::Mat& rvec, const cv::Mat& tvec)
{
  cv::Mat point(3, 1, CV_64F);
  point.at<double>(0) = pos_in_field.x;
  point.at<double>(1) = pos_in_field.y;
  point.at<double>(2) = pos_in_field.z;

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Mat camera_point = R * point + tvec;
  return cv::Point3f(camera_point.at<double>(0), camera_point.at<double>(1), camera_point.at<double>(2));
}

cv::Point2f fieldToImg(const cv::Point3f& pos_in_field, const CameraMetaInformation& camera_information)
{
  if (!camera_information.has_camera_parameters() || !camera_information.has_pose())
  {
    throw std::runtime_error(HL_DEBUG + " camera_information is not fully specified");
  }
  cv::Mat camera_matrix, distortion_coeffs, rvec, tvec;
  cv::Size size;
  intrinsicToCV(camera_information.camera_parameters(), &camera_matrix, &distortion_coeffs, &size);
  pose3DToCV(camera_information.pose(), &rvec, &tvec);
  std::vector<cv::Point3f> object_points = { pos_in_field };
  std::vector<cv::Point2f> img_points;

  if (isPointValidForCorrection(pos_in_field, rvec, tvec, camera_matrix, distortion_coeffs))
  {
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coeffs, img_points);
  }
  else
  {
    throw std::runtime_error("The point is too far from the center to be corrected.");
  }
  return img_points[0];
}

bool isPointValidForCorrection(const cv::Point3f& pos, cv::Mat rvec, cv::Mat tvec, cv::Mat camera_matrix,
                               cv::Mat distortion_coeffs)
{
  cv::Mat no_distortion;
  std::vector<cv::Point3f> object_points = { pos };
  std::vector<cv::Point2f> img_points_theoretical;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, no_distortion, img_points_theoretical);

  double focal_x = camera_matrix.at<double>(0, 0);
  double focal_y = camera_matrix.at<double>(1, 1);
  double center_x = camera_matrix.at<double>(0, 2);
  double center_y = camera_matrix.at<double>(1, 2);
  cv::Point3f normalized((img_points_theoretical[0].x - center_x) / focal_x,
                         (img_points_theoretical[0].y - center_y) / focal_y, 1.0);

  if (normalized.z != 1.0)
  {
    throw std::runtime_error(" pos.z should be equal to 1.0 instead of " + std::to_string(pos.z));
  }

  if (distortion_coeffs.at<double>(0, 2) != 0 or distortion_coeffs.at<double>(0, 3) != 0 or
      distortion_coeffs.at<double>(0, 4) != 0)
  {
    return true;
  }
  // k3, p1 and p2 are equal to 0

  double x = static_cast<double>(normalized.x);
  double y = static_cast<double>(normalized.y);

  // we can suppose that x and y are positive
  if (x < 0)
  {
    x = -x;
  }
  if (y < 0)
  {
    y = -y;
  }

  // if the point is close to the center it is valid
  if (x < 0.01 and y < 0.01)
  {
    return true;
  }
  else
  {
    double k1 = distortion_coeffs.at<double>(0, 0);
    double k2 = distortion_coeffs.at<double>(0, 1);

    if (k1 == 0 and k2 == 0)  // there is no distortion
    {
      return true;
    }

    if (k2 == 0 and k1 > 0)  // f' is always positive
    {
      return true;
    }

    double u = x + y;
    double p = x / u;
    double q = y / u;

    double a = 5 * k2 * std::pow(p * p + q * q, 2);
    double b = 3 * k1 * (p * p + q * q);
    double c = 1;

    if (k2 == 0 and k1 < 0)  // the roots of f' are +- 1/sqrt(3*(-k1)*(p^2+q^2))
    {
      return u * u < 1 / (-b);
    }

    // in the following k2 is non zero

    double disc = b * b - 4 * a * c;

    if (disc <= 0)  // the sign of g is constant, hence f is monotonic
    {
      return true;
    }
    else
    {
      double u1 = (-b - std::sqrt(disc)) / (2 * a);
      double u2 = (-b + std::sqrt(disc)) / (2 * a);

      if (u2 < u1)
      {
        double tmp = u1;
        u1 = u2;
        u2 = tmp;
      }

      // in the following u1 < u2

      if (u2 < 0)  // both are negative, hence the roots of f' are complexe
      {
        return true;
      }
      else if (u1 > 0)  // u1 is the smallest positivie root
      {
        return u * u < u1;
      }
      else  // u2 is the smallest positivie root
      {
        return u * u < u2;
      }
    }
  }
}

uint64_t getTS(const FrameEntry& entry, bool utc)
{
  if (utc)
  {
    if (!entry.has_utc_ts())
    {
      throw std::logic_error(HL_DEBUG + " no utc ts available");
    }
    return entry.utc_ts();
  }
  if (!entry.has_monotonic_ts())
  {
    throw std::logic_error(HL_DEBUG + " no monotonic ts available");
  }
  return entry.monotonic_ts();
}

int getIndex(const VideoMetaInformation& meta_information, uint64_t time_stamp, bool utc)
{
  for (int idx = 0; idx < meta_information.frames_size(); idx++)
  {
    uint64_t frame_ts = getTS(meta_information.frames(idx), utc);
    if (frame_ts == time_stamp)
      return idx;
    if (frame_ts > time_stamp)
      return idx - 1;
  }
  return meta_information.frames_size() - 1;
}

uint64_t getTimeStamp(const VideoMetaInformation& meta_information, int index, bool utc)
{
  if (meta_information.frames_size() <= index || index < 0)
    throw std::out_of_range(HL_DEBUG + " invalid index: " + std::to_string(index) +
                            " (nb frames: " + std::to_string(meta_information.frames_size()) + ")");
  const FrameEntry& frame = meta_information.frames(index);
  return getTS(frame, utc);
}

cv::Point2f protobufToCV(const Point2DMsg& msg)
{
  return cv::Point2f(msg.x(), msg.y());
}

cv::Point3f protobufToCV(const Point3DMsg& msg)
{
  return cv::Point3f(msg.x(), msg.y(), msg.z());
}

void cvToProtobuf(const cv::Point2f& pos, Point2DMsg* msg)
{
  msg->set_x(pos.x);
  msg->set_y(pos.y);
}

void cvToProtobuf(const cv::Point3f& pos, Point3DMsg* msg)
{
  msg->set_x(pos.x);
  msg->set_y(pos.y);
  msg->set_z(pos.z);
}

void protobufToCV(const std::vector<Match2D3DMsg>& matches, std::vector<cv::Point2f>* img_pos,
                  std::vector<cv::Point3f>* obj_pos)
{
  img_pos->clear();
  obj_pos->clear();
  for (const Match2D3DMsg& match : matches)
  {
    img_pos->push_back(protobufToCV(match.img_pos()));
    obj_pos->push_back(protobufToCV(match.obj_pos()));
  }
}

bool fieldToImg(const cv::Point3f& pos_in_field, const CameraMetaInformation& camera_information, cv::Point2f* img_pos)
{
  cv::Mat rvec, tvec;
  pose3DToCV(camera_information.pose(), &rvec, &tvec);
  try
  {
    *img_pos = fieldToImg(pos_in_field, camera_information);
  }
  catch (std::runtime_error)
  {
    return false;
  }
  cv::Point3f camera_point = fieldToCamera(pos_in_field, rvec, tvec);
  cv::Size img_size = getImgSize(camera_information);
  return camera_point.z > 0 && img_pos->x >= 0 && img_pos->y >= 0 && img_pos->x < img_size.width &&
         img_pos->y < img_size.height;
}

Json::Value file2Json(const std::string& path)
{
  std::ifstream in(path);
  if (!in.good())
  {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  Json::Value root;
  in >> root;
  return root;
}

void writeJson(const Json::Value& v, const std::string& path, bool human)
{
  std::string content = json2String(v, human);
  std::ofstream output(path);
  if (!output.good())
  {
    throw std::runtime_error(HL_DEBUG + "failed to open file at '" + path + "'");
  }
  output << content;
}

std::string json2String(const Json::Value& v, bool human)
{
  if (human)
  {
    return Json::StyledWriter().write(v);
  }
  return Json::FastWriter().write(v);
}

void checkMember(const Json::Value& v, const std::string& key)
{
  if (!v.isObject() || !v.isMember(key))
  {
    throw std::runtime_error(HL_DEBUG + "Could not find member '" + key + "'");
  }
}

template <>
Json::Value val2Json<bool>(const bool& val)
{
  return Json::Value(val);
}

template <>
Json::Value val2Json<int>(const int& val)
{
  return Json::Value(val);
}

template <>
Json::Value val2Json<size_t>(const size_t& val)
{
  return Json::Value((int)val);
}

template <>
Json::Value val2Json<float>(const float& val)
{
  return Json::Value(val);
}

template <>
Json::Value val2Json<double>(const double& val)
{
  return Json::Value(val);
}

template <>
Json::Value val2Json<std::string>(const std::string& val)
{
  return Json::Value(val);
}

template <>
Json::Value val2Json<cv::Scalar>(const cv::Scalar& color)
{
  Json::Value v;
  for (int i = 0; i < 3; i++)
  {
    v[i] = color[i];
  }
  return v;
}

template <>
void readVal<bool>(const Json::Value& v, const std::string& key, bool* dst)
{
  checkMember(v, key);
  if (!v[key].isBool())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting a bool for key '" + key + "'");
  }
  *dst = v[key].asBool();
}

template <>
void readVal<int>(const Json::Value& v, const std::string& key, int* dst)
{
  checkMember(v, key);
  if (!v[key].isInt())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting an int for key '" + key + "'");
  }
  *dst = v[key].asInt();
}

template <>
void readVal<float>(const Json::Value& v, const std::string& key, float* dst)
{
  checkMember(v, key);
  if (!v[key].isDouble())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting a double for key '" + key + "'");
  }
  *dst = v[key].asDouble();
}

template <>
void readVal<double>(const Json::Value& v, const std::string& key, double* dst)
{
  checkMember(v, key);
  if (!v[key].isDouble())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting a double for key '" + key + "'");
  }
  *dst = v[key].asDouble();
}

template <>
void readVal<std::string>(const Json::Value& v, const std::string& key, std::string* dst)
{
  checkMember(v, key);
  if (!v[key].isString())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting a string for key '" + key + "'");
  }
  *dst = v[key].asString();
}

template <>
void readVal<cv::Scalar>(const Json::Value& v, const std::string& key, cv::Scalar* dst)
{
  checkMember(v, key);
  if (!v[key].isArray())
  {
    throw std::runtime_error(HL_DEBUG + "Expecting an array for '" + key + "'");
  }
  if (v[key].size() != 3)
  {
    throw std::runtime_error(HL_DEBUG + "Invalid size for array at '" + key + "', expecting 3");
  }
  for (Json::ArrayIndex idx = 0; idx < 3; idx++)
  {
    if (!v[key][idx].isInt())
    {
      throw std::runtime_error(HL_DEBUG + "Invalid type for " + key + "[" + std::to_string(idx) + "], expecting int");
    }
    int val = v[key][idx].asInt();
    if (val < 0 || val > 255)
    {
      throw std::runtime_error(HL_DEBUG + "Invalid value for " + key + "[" + std::to_string(idx) +
                               "]: range is [0,255]");
    }
    (*dst)[idx] = (char)val;
  }
}

std::ostream& operator<<(std::ostream& out, const RobotIdentifier& id)
{
  return out << "{team: " << id.team_id() << ", robot: " << id.robot_id() << "}";
}

std::ostream& operator<<(std::ostream& out, const RobotCameraIdentifier& id)
{
  return out << "{robot: " << id.robot_id() << ", camera: " << id.camera_name() << "}";
}

std::ostream& operator<<(std::ostream& out, const VideoSourceID& id)
{
  if (id.has_robot_source())
    out << "{robot: " << id.robot_source();
  else if (id.has_external_source())
    out << "{camera:" << id.external_source();
  else
    throw std::logic_error(HL_DEBUG + "incomplete message");
  if (id.has_utc_start())
  {
    out << ", utc_start: " << id.utc_start();
  }
  return out << "}";
}

}  // namespace hl_communication
