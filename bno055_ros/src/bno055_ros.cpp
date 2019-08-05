/** Timming lib */
#include <bno055_ros/bno055_ros.hpp>

bno055_arduino::bno055_arduino(ros::NodeHandle &n_imu)
{
    device_ = "/dev/ttyACM0";
    imu_publisher = n_imu.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
    mag_publisher = n_imu.advertise<sensor_msgs::MagneticField>("imu/mag", 2);
    imu_sequence = 0;
}

int bno055_arduino::openPort()
{
    try
    {
        Serial_port.Open(device_.c_str()); // Open the Serial Port at the desired hardware port.
    }
    catch (const LibSerial::OpenFailed &)
    {
        std::cerr << "The serial port did not open correctly." << std::endl;
        return EXIT_FAILURE;
    }
    Serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_460800);             // Set the baud rate of the serial port.
    Serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);   // Set the number of data bits.
    Serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE); // Turn off hardware flow control.
    Serial_port.SetParity(LibSerial::Parity::PARITY_NONE);                 // Disable parity.
    Serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);             // Set the number of stop bits.
}

void bno055_arduino::update()
{
    std::string buffer;
    std::string delimiter = ";";
    std::array<std::string, 13> imu_data;
    size_t pos = 0;
    Serial_port.ReadLine(buffer, '\n', 30);
    for (int idex = 0; idex < 13; idex++)
    {
        pos = buffer.find(delimiter);
        imu_data[idex] = buffer.substr(0, pos);
        buffer.erase(0, pos + delimiter.length());
    }
    imu_sequence++;
    ros_time = ros::Time::now();
    imu_msg.header.stamp = ros_time;
    imu_msg.header.seq = imu_sequence;
    imu_msg.linear_acceleration.x = stod(imu_data[0]) * g_ * 0.001;
    imu_msg.linear_acceleration.y = stod(imu_data[1]) * g_ * 0.001;
    imu_msg.linear_acceleration.z = stod(imu_data[2]) * g_ * 0.001;
    imu_msg.angular_velocity.x = stod(imu_data[3]) * M_PI / 180;
    imu_msg.angular_velocity.y = stod(imu_data[4]) * M_PI / 180;
    imu_msg.angular_velocity.z = stod(imu_data[5]) * M_PI / 180;
    imu_msg.orientation.x = stod(imu_data[9]);
    imu_msg.orientation.y = stod(imu_data[10]);
    imu_msg.orientation.z = stod(imu_data[11]);
    imu_msg.orientation.w = stod(imu_data[12]);
    mag_msg.header.stamp = imu_msg.header.stamp;
    mag_msg.header.seq = imu_sequence;
    mag_msg.magnetic_field.x = stod(imu_data[6]);
    mag_msg.magnetic_field.y = stod(imu_data[7]);
    mag_msg.magnetic_field.z = stod(imu_data[8]);
    imu_publisher.publish(imu_msg);
    mag_publisher.publish(mag_msg);
}

void bno055_arduino::closePort()
{
    Serial_port.Close();
}
