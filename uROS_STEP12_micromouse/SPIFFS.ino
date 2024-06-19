// Copyright 2023 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

void mapWrite(void)
{
  String file_tmp;
  unsigned char data_temp;
  file_tmp = "/map.txt";

  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();

  Serial.printf("\n\r map_data write ");
  Serial.println(file_tmp);

  File file = SPIFFS.open(file_tmp, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      data_temp = g_map_control.getWallData(i, j, north) +
                  (g_map_control.getWallData(i, j, east) << 2) +
                  (g_map_control.getWallData(i, j, south) << 4) +
                  (g_map_control.getWallData(i, j, west) << 6);
      if (file.write(data_temp)) {  //バイナリ書き込み
      } else {
        Serial.println("- write failed");
      }
    }
  }

  file.close();
  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}

void copyMap(void)
{
  String file_tmp;
  unsigned char read_data;

  controlInterruptStop();
  sensorInterruptStop();
  PWMInterruptStop();

  File file = SPIFFS.open("/map.txt", FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.println("- failed to open file for reading");
    return;
  }
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      if (file.available()) {
        read_data = file.read();
        g_map_control.setWallData(i, j, north, read_data & 0x03);
        g_map_control.setWallData(i, j, east, (read_data >> 2) & 0x03);
        g_map_control.setWallData(i, j, south, (read_data >> 4) & 0x03);
        g_map_control.setWallData(i, j, west, (read_data >> 6) & 0x03);
      } else {
        Serial.println("Read Error");
      }
    }
  }
  file.close();
  controlInterruptStart();
  sensorInterruptStart();
  PWMInterruptStart();
}
