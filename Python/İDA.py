import os
import time
import random
from pyfiglet import Figlet
from colorama import Fore, Style, init

# Terminal renk desteği başlat
init(autoreset=True)

RENKLER = [Fore.RED, Fore.GREEN, Fore.YELLOW, Fore.BLUE, Fore.MAGENTA, Fore.CYAN, Fore.WHITE]

def temizle():
    os.system('cls' if os.name == 'nt' else 'clear')

def renkli_yaz(metin):
    return random.choice(RENKLER) + metin + Style.RESET_ALL

def yukleniyor():
    for i in range(12):  # Daha uzun bekleme
        temizle()
        print(renkli_yaz("Yükleniyor" + "." * (i % 4)))
        time.sleep(0.3)  # Daha yavaş

def hareketli_cizgi():
    uzunluk = 25
    for i in range(uzunluk):
        temizle()
        print(renkli_yaz(" " * i + ">>> Hazırlanıyor <<<"))
        time.sleep(0.08)  # Daha yavaş hareket

def buyuk_yazi(metin):
    f = Figlet(font='slant')
    yaz = f.renderText(metin)
    for satir in yaz.split("\n"):
        print(renkli_yaz(satir))
        time.sleep(0.07)  # Daha yavaş satır satır yazdırma

# Program akışı
yukleniyor()
hareketli_cizgi()

# Önce "Deneyap"
temizle()
buyuk_yazi("Deneyap")
time.sleep(2)  # Bekleme

# Sonra "İDA"
temizle()
buyuk_yazi("IDA")
time.sleep(60)  # Finalde ekranda uzun süre kalsın

temizle()
print(renkli_yaz("Geleceği Kodlayan Gençler!"))
