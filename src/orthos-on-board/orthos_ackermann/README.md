Empfängt Daten aus ```cmd_vel``` und verarbeitet diese. Die vom PS3 Controller gesendeten Werte, die maximal +1 und -1 sein können, werden im zuerst im Winkel gespeichert. Dort werden die Werte in Radian umgewandelt und so skaliert, dass der Winkel des virtuellen inneren Rades maximal +-30° beträgt.

Nun findet die Berechnung der Winkel der vier realen Räder nach Ackerman-Steering statt.

Der Winkel der einzelnen Räder wird als Radian in die Topics ```orthos_position_controller_{left, right}_{front, rear}/command``` als Float64 gepublisht.

Die Geschwindigkeiten der einzelnen Räder werden ebenfalls berechnet und mit einem Skalierungsfaktor versehen in die Topics ```orthos_velocity_controller_{1..6}``` gepublisht.

Am Ende werden Transforms der einzelnen Teilstränge des Roboters berechnet, um diesen in RVIZ zu visualisieren.
