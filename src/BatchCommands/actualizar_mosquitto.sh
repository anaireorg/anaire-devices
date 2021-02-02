# Actualizar a la última versón de sw
# for id in 14745c 4ad440 1474dc 4b9be5 4c2bb1 fcce55 efae33; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"update": "ON"}'; done

# Reiniciar 
for id in 14745c 4ad440 1474dc 4b9be5 4c2bb1 fcce55 efae33; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"reboot": "ON"}'; done

# Factory reset
# for id in 14745c 4ad440 1474dc 4b9be5 4c2bb1 fcce55 efae33; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"reset": "ON"}'; done

# // Calibrar
#for id in 14745c 4ad440 1474dc 4b9be5 4c2bb1 fcce55 efae33; do mosquitto_pub -h portal.anaire.org -p 30183 -q 1 -t config/$id -m '{"FRC": "ON"}'; done

