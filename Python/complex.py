import random

def oyun():
    hedef = random.randint(1, 100)
    tahmin_sayisi = 0
    tahmin_gecmisi = []

    print("1 ile 100 arasında bir sayı tuttum. Bakalım tahmin edebilecek misin?")

    while True:
        try:
            tahmin = int(input("Tahmininiz: "))
        except ValueError:
            print("Lütfen sayı girin!")
            continue

        tahmin_sayisi += 1
        tahmin_gecmisi.append(tahmin)

        if tahmin < hedef:
            print("Daha büyük bir sayı söyle.")
        elif tahmin > hedef:
            print("Daha küçük bir sayı söyle.")
        else:
            print(f"Tebrikler! {tahmin_sayisi} denemede buldunuz.")
            break

    # Tahmin geçmişini dosyaya yaz
    with open("tahminler.txt", "w") as f:
        f.write("Tahminleriniz: " + ", ".join(map(str, tahmin_gecmisi)) + "\n")
        f.write(f"Toplam deneme: {tahmin_sayisi}\n")
        f.write(f"Hedef sayı: {hedef}\n")

    print("Tahmin geçmişiniz 'tahminler.txt' dosyasına kaydedildi.")

# Oyunu başlat
oyun()
