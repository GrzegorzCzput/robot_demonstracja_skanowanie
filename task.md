Zadanie: Przygotowanie robota TRLC-DK1 do demo
Cel
Przygotowanie robota TRLC-DK1 do demonstracji symulowanego procesu skanowania 3D za pomocą telefonu zamontowanego na końcu manipulatora.
Robot i skanowany obiekt (prawdopodobnie średniej wielkości zabawka 20 × 20 × 20 cm) będą znajdować się na tym samym stole.
Oczekiwane zachowanie

Robot wykonuje zapętloną sekwencję ruchów pomiędzy około 4 pozycjami obserwacyjnymi z różnych stron obiektu.
W każdej pozycji kamera telefonu jest skierowana na ten sam punkt - przybliżony środek skanowanego obiektu, prawdopodobnie pod lekkim kątem w dół.
Robot zatrzymuje się na krótko (około 1–2 sekundy) w każdej pozycji, symulując wykonanie zdjęcia.
Przejścia między pozycjami powinny być płynne, bez gwałtownych ruchów.
Po ostatniej pozycji robot płynnie wraca do pierwszej i kontynuuje sekwencję.
Nie wykonujemy rzeczywistych zdjęć ani skanowania - chodzi wyłącznie o wizualną demonstrację procesu.
Sekwencja powinna działać autonomicznie, bez konieczności ingerencji operatora.
Zakres prac

Uruchomienie i konfiguracja sterowania robotem.
Przygotowanie mocowania telefonu Galaxy A26 5G na manipulatorze.
Zaplanowanie i zaprogramowanie pozycji oraz trajektorii ruchu.
Dopasowanie pozycji kamery do ustalonego położenia obiektu na stole.
Implementacja automatycznej pętli oraz możliwości uruchomienia i zatrzymania demonstracji.
Sprawdzenie bezpieczeństwa ruchu, w tym unikania kolizji z obiektem, stołem i elementami stanowiska.
Przetestowanie stabilności i powtarzalności działania podczas dłuższej pracy.
Dodatkowo (mile widziane, ale nieobowiązkowe)

Możliwość łatwej zmiany punktu w przestrzeni 3D (współrzędne X, Y, Z), na który skierowana jest kamera telefonu.
Idealnie, gdyby po zmianie tego punktu robot automatycznie dostosowywał orientację kamery w poszczególnych pozycjach obserwacyjnych, bez konieczności ich ręcznego programowania od nowa.
Oczekiwany rezultat
Gotowy do uruchomienia robot z zamontowanym telefonem, wykonujący płynną, powtarzalną i bezpieczną sekwencję symulowanego skanowania 3D.
Zależy nam przede wszystkim na niezawodności i atrakcyjnym wizualnie ruchu. Sposób implementacji pozostawiam Wam - nie ma potrzeby budowania bardziej skomplikowanego systemu, niż wymaga tego demonstracja.