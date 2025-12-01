def fibonacci(n):
    a, b = 0, 1
    for _ in range(n):
        a, b = b, a + b
    return a

# Örnek kullanım
for i in range(10):
    print(f"{i}. sayı: {fibonacci(i)}")
