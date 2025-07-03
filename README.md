Celem niniejszej pracy było zaprojektowanie oraz implementacja zintegrowanego systemu sterowania dostępem, przeznaczonego do zastosowania w środowisku inteligentnego domu. System ten opiera się na wykorzystaniu nowoczesnych mikrokontrolerów STM32 oraz ESP32, które odpowiadają za obsługę elementów wejściowych i komunikacyjnych, takich jak czytnik RFID, moduły WiFi oraz kamera, wraz z algorytmem detkecji obecności osób opartym o model sieci neuronowej YOLO.

W ramach projektu zrealizowano dwupoziomowy mechanizm autoryzacji dostępu:
\begin{itemize}
    \item Pierwszy poziom oparty jest na technologii RFID – umożliwia on identyfikację użytkownika poprzez zbliżenie karty lub breloka do czytnika, co jest interpretowane przez mikrokontroler STM32 i dalej przetwarzane przez system.
    \item Drugi poziom wykorzystuje algorytm rozpoznawania obiektów YOLO, uruchomione na komputerze odbierającym strumień obrazu z kamery ESP32-CAM. Detekcja obecności człowieka przy drzwiach umożliwia powiadomienie użytkownika za pośrednictwem aplikacji mobilnej, a dzięki przesłanemu obrazowi z kamery użytkownik może podjąć decyzję o otwarciu drzwi.
\end{itemize}

Zawartość repozytorium:
- main.c - plik główny projektu w STM32CubeIDE odpowiedzialny za obsługę czytnika RFID, serwomechanizmu imitujacego zamek w drzwiach oraz wyświetlacz LCD i wysyłanie informacji przez UART do ESP32-C6.
- mqtt_esp32.ino - plik środowiska ArduinoIDE napisany dla ESP32-C6 odpowiedzialny za odbiór poleceń z STM32 i wysyłanie ich przez MQTT do aplikacji mobilnej oraz poleceń z aplikacji do STM32.
- Folder "CameraWebSerwer" - projekt ArduinoIDE napisany dla ESP32-CAM obsługujący strumieniowanie obrazu z kamery na serwer internetowy oraz wysyłanie wiadomości o naciśniętym przycisku "dzwonka" przez MQTT do aplikacji mobilnej.
- Read-YOLO-Sent.py - plik Python zawierający analizę obrazu z kamery przy pomocy modułu YOLO i gdy został wykryty człowiek, wysłanie zdjęcia i informacji przez MQTT do aplikacji mobilnej oraz zapis zdjęć w archiwum PC.
