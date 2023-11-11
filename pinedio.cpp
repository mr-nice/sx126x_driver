/*
    This file is part of SX126x Portable driver.
    Copyright (C) 2020 ReimuNotMoe

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#define ADV_DEBUG
#include "SX126x.hpp"
#include <cassert>
#include <cinttypes>
#include <cstdio>
#include <fcntl.h>
#include <iostream>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <gpiod.h>
#include <stdio.h>
int spi;

const char *chipname = "gpiochip0";
struct gpiod_chip *chip;
struct gpiod_line *line_11; // slct 
struct gpiod_line *line_16; // ini

class JFSx1262 : public SX126x {
public:
  uint8_t HalGpioRead(GpioPinFunction_t func) override {
    uint8_t value = gpiod_line_get_value(line_11);
    //std::cout << "read: " << std::hex << value;
    printf("read |%x|\n", value);
    return value;
  }

  void HalGpioWrite(GpioPinFunction_t func, uint8_t value) override {
    std::cout << "GPIO WRITE << " << func << " -- " << std::to_string(value) << std::endl;

    if(func != GpioPinFunction_t::GPIO_PIN_RESET) {
      std::cout << "ERROR" << std::endl;
      throw;
    }
    printf("write |%x|\n", value);
    gpiod_line_set_value(line_16, value);
  }

  void HalSpiTransfer(uint8_t *buffer_in, const uint8_t *buffer_out,
                      uint16_t size) override {
    const uint8_t *mosi = buffer_out; // output data
    uint8_t *miso = buffer_in; // input data

    struct spi_ioc_transfer spi_trans;
    memset(&spi_trans, 0, sizeof(spi_trans));

    spi_trans.tx_buf = (unsigned long) mosi;
    spi_trans.rx_buf = (unsigned long) miso;
    spi_trans.cs_change = true;
    spi_trans.len = size;

    /*int status = */ioctl (spi, SPI_IOC_MESSAGE(1), &spi_trans);
  }
};

void Print(const SX126x::RadioStatus_t status) {
  std::cout << "Status : " << std::to_string(status.Value) << "\n";
  std::cout << "\tCmd : " << std::to_string(status.Fields.CmdStatus) << "\n";
  std::cout << "\tCpu : " << std::to_string(status.Fields.CpuBusy) << "\n";
  std::cout << "\tChip : " << std::to_string(status.Fields.ChipMode) << "\n";
}

void Print(const SX126x::RadioError_t error) {
  std::cout << "Error : " << std::to_string(error.Value) << "\n";
}

uint8_t data[100] = {0x55, 0xBB,0x55, 0xAA,0x55, 0xAA,0x55, 0xAA};


void init_gpiod()
{
  chip = gpiod_chip_open_by_name(chipname);
  line_11 = gpiod_chip_get_line(chip, 11);
  gpiod_line_request_input(line_11, "sclt");
  line_16 = gpiod_chip_get_line(chip, 16);
  gpiod_line_request_output(line_16, "ini", 0);
}

void exit_gpiod()
{
  gpiod_line_release(line_11);
  gpiod_line_release(line_16);
}

int main() {
  for(int i = 0; i< 100; i++) {
    data[i] = i;
  }

  init_gpiod();

  spi = open("/dev/spidev0.0", O_RDWR);
  uint8_t mmode = SPI_MODE_0;
  uint8_t lsb = 0;
  ioctl(spi, SPI_IOC_WR_MODE, &mmode);
  ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);

  JFSx1262 radio;
  radio.Reset();
  auto status = radio.GetStatus();
  Print(status);
  radio.SetDeviceType(SX126x::DeviceType_t::SX1262);
  auto error = radio.GetDeviceErrors();
  Print(error);

  radio.SetStandby(SX126x::STDBY_RC);
  //radio.SetDio3AsTcxoCtrl(SX126x::RadioTcxoCtrlVoltage_t::TCXO_CTRL_3_3V, 5 << 6);

  SX126x::CalibrationParams_t calibrationParams;
  calibrationParams.Fields.ADCBulkNEnable = 1;
  calibrationParams.Fields.ADCBulkPEnable = 1;
  calibrationParams.Fields.ImgEnable = 1;
  calibrationParams.Fields.ADCPulseEnable = 1;
  calibrationParams.Fields.PLLEnable = 1;
  calibrationParams.Fields.RC13MEnable  = 1;
  calibrationParams.Fields.RC64KEnable = 1;
  radio.Calibrate(calibrationParams);

  //radio.SetDio2AsRfSwitchCtrl(true);


  radio.SetStandby(SX126x::STDBY_XOSC);
  radio.SetRegulatorMode(SX126x::RadioRegulatorMode_t::USE_DCDC);
  radio.SetBufferBaseAddresses(0,0);
  radio.SetTxParams(22, SX126x::RADIO_RAMP_3400_US);
  radio.SetRfFrequency(868000000);

  radio.SetStopRxTimerOnPreambleDetect(false);
  radio.SetLoRaSymbNumTimeout(0);
  radio.SetPacketType(SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA);
  radio.SetDioIrqParams(0xffff, 0x0001, 0, 0);


  SX126x::ModulationParams_t modulationParams;
  modulationParams.PacketType = SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.SpreadingFactor = SX126x::RadioLoRaSpreadingFactors_t::LORA_SF12;
  modulationParams.Params.LoRa.CodingRate =  SX126x::RadioLoRaCodingRates_t::LORA_CR_4_5;
  modulationParams.Params.LoRa.Bandwidth = SX126x::RadioLoRaBandwidths_t::LORA_BW_500;
  modulationParams.Params.LoRa.LowDatarateOptimize = false;
  radio.SetModulationParams(modulationParams);

  SX126x::PacketParams_t packetParams;
  packetParams.PacketType = SX126x::RadioPacketTypes_t::PACKET_TYPE_LORA;
  packetParams.Params.LoRa.PreambleLength = 100;
  packetParams.Params.LoRa.HeaderType = SX126x::RadioLoRaPacketLengthsMode_t::LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.PayloadLength = 100;
  packetParams.Params.LoRa.CrcMode = SX126x::RadioLoRaCrcModes_t::LORA_CRC_OFF;
  packetParams.Params.LoRa.InvertIQ = SX126x::RadioLoRaIQModes_t::LORA_IQ_NORMAL;
  radio.SetPacketParams(packetParams);

  auto mode = radio.GetOperatingMode();
  std::cout << "Mode  : " << std::to_string(mode) << std::endl;

  //radio.SetRx(0xffff);
  mode = radio.GetOperatingMode();
  std::cout << "Mode  : " << std::to_string(mode) << std::endl;

  radio.SetPacketParams(packetParams);
  radio.WriteBuffer(0, data, 100);

  uint8_t readData[8];
  radio.ReadBuffer(0, readData, 8);
  for(int i = 0; i < 8; i++) {
    printf("0x%x ", readData[i]);
  }
  printf("\n");


  int c = 0;

    radio.SetTx(0xffffffff);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    error = radio.GetDeviceErrors();
    Print(error);

    while(1) {


    radio.ProcessIrqs();
    auto mode = radio.GetOperatingMode();
    std::cout << "Mode  : " << std::to_string(mode) << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  exit_gpiod();
  return 0;
}
