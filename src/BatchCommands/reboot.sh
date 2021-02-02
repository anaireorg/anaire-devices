#!/bin/bash

dispositivos=( 14745c 1473c0 147874 e4ed60 146eaf 24719 fcc99a 147b92 147a20 1474dc 4b9be5 4c2bb1 fcce55 14788e 24e49 141240 24826 fc9cbc 85e646 1d48d 2205b e482e5 4a9865 8625d4 147a11 fca9ac efae33 224cb 147def 1c775 22550 fd2ae9 222bf af01ca 4ad440 )

# // Calibrar
for id in "${dispositivos[@]}"; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"reboot": "ON"}'; done

