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



void listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}


void map_write(void) {
  String file_tmp;
  unsigned char data_temp;
  file_tmp = "/map.txt";

  ControlInterruptStop();
  SensorInterruptStop();
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
      data_temp = map_control.get_wall_data(i, j, north) + (map_control.get_wall_data(i, j, east) << 2) + (map_control.get_wall_data(i, j, south) << 4) + (map_control.get_wall_data(i, j, west) << 6);
      if (file.write(data_temp)) {  //バイナリ書き込み
      } else {
        Serial.println("- write failed");
      }
    }
  }

  file.close();
  ControlInterruptStart();
  SensorInterruptStart();
  PWMInterruptStart();
}

void copy_map(void) {
  String file_tmp;
  unsigned char read_data;

  ControlInterruptStop();
  SensorInterruptStop();
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
        map_control.set_wall_data(i, j, north, read_data & 0x03);
        map_control.set_wall_data(i, j, east, (read_data >> 2) & 0x03);
        map_control.set_wall_data(i, j, south, (read_data >> 4) & 0x03);
        map_control.set_wall_data(i, j, west, (read_data >> 6) & 0x03);
      } else {
        Serial.println("Read Error");
      }
    }
  }
  file.close();
  ControlInterruptStart();
  SensorInterruptStart();
  PWMInterruptStart();
}
