#include "gauss_driver/rpi/rpi_diagnostics.h"

RpiDiagnostics::RpiDiagnostics()
{
    cpu_temperature = 0;

    startReadingData();
}

int RpiDiagnostics::getRpiCpuTemperature()
{
    return cpu_temperature;
}

void RpiDiagnostics::readCpuTemperature()
{
#ifdef __arm__
    std::fstream cpu_temp_file("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);
    
    int read_temp;
    cpu_temp_file >> read_temp;
    if (read_temp > 0) {
        cpu_temperature = read_temp / 1000;
    }
#endif
}

void RpiDiagnostics::startReadingData()
{
   read_hardware_data_thread.reset(new std::thread(boost::bind(&RpiDiagnostics::readHardwareDataLoop, this))); 
}

void RpiDiagnostics::readHardwareDataLoop()
{
    double read_rpi_diagnostics_frequency;
    ros::param::get("~read_rpi_diagnostics_frequency", read_rpi_diagnostics_frequency);
    ros::Rate read_rpi_diagnostics_rate = ros::Rate(read_rpi_diagnostics_frequency);

    while (ros::ok()) {
        readCpuTemperature();

        // check if Rpi is too hot
        if (cpu_temperature > 75) {
            ROS_ERROR("Rpi temperature is really high !");
        }
        if (cpu_temperature > 85) {
            ROS_ERROR("Rpi is too hot, shutdown to avoid any damage");
            std::system("sudo shutdown now");
        }

        read_rpi_diagnostics_rate.sleep();
    }
}
