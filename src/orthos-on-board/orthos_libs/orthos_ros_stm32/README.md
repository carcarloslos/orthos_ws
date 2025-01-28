Mit disem Paket können Bytes mit dem STM32 ausgetauscht werden. Über die Bibliothek [LibSerial](https://github.com/crayzeewulf/libserial) werden Bytes via UART von der Jetson über Port ttyS0 übertragen. 

Damit diese übertragen werden können, muss sichergestellt werden, dass /dev/ttyS0 (in Linux ist alles eine Datei, sogar die angeschlossenen Geräte!) die richtigen Berechtigungen (lesen, schreiben) hat. 
Dies kann mit `ls -l` überprüft und mit `sudo chmod o+rw /dev/ttyS0` gesetzt werden. 

Um einfache Dinge in /dev/ttyS0 zu schreiben, kann zum Beispiel `echo` genutzt werden. Mehr  [hier](https://unix.stackexchange.com/questions/117037/how-to-send-data-to-a-serial-port-and-see-any-answer)

Das Paket empfängt Daten der Topics ```orthos_position_controller_{left, right}_{front, rear}/command```, wandelt die Winkel in Grad um und endet diese an den STM32. 

Dazu werden zuerst die Motoren aktiviert und dann ein Buffer mit 9 Byte gefüllt (COMMAND_BYTE, dann 4x je 2 Byte mit Winkelinformationen)

### Signale zum Steuern des STM32
byte | Variable | Beschreibung
------------ | ------------- | -------------
10000001 | **PWM_START_BYTE** | Aktivieren der Motoren
10000010 | **COMMAND_BYTE** | Hiernach Folgen Anweisungen für die Motoren (je 2 Byte, erst Dezimal, dann Nachkommateil)
10000100 | **LED_BYTE** | Tooglet die LED (zu Testzwecken)
10000011 | **PWM_STOP_BYTE** | Deaktiviert die Motoren

