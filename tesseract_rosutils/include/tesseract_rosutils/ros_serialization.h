#ifndef TESSERACT_ROSUTILS_ROS_SERIALIZATION_H
#define TESSERACT_ROSUTILS_ROS_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/serialization.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_rosutils
{
template <typename MessageType>
inline bool toFile(const std::string& filepath, const MessageType& msg)
{
  std::ofstream ofs(filepath, std::ios::out | std::ios::binary);

  uint32_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, msg);
  ofs.write((char*)obuffer.get(), serial_size);
  ofs.close();

  return true;
}

template <typename MessageType>
inline MessageType fromFile(const std::string& filepath)
{
  std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  auto file_size = static_cast<long>(end - begin);
  boost::shared_array<uint8_t> ibuffer(new uint8_t[static_cast<unsigned long>(file_size)]);
  ifs.read((char*)ibuffer.get(), file_size);
  ros::serialization::IStream istream(ibuffer.get(), static_cast<uint32_t>(file_size));

  MessageType msg;
  ros::serialization::deserialize(istream, msg);
  ifs.close();

  return msg;
}

}  // namespace tesseract_rosutils
#endif  // TESSERACT_ROSUTILS_ROS_SERIALIZATION_H
