#include <stdio.h>
#include <math.h>

int thuaso(int n, int k) {
    int cnt = 0;
    
    if (n < 2) return -1;  // Không có th?a s? nguyên t? n?u n < 2
    
    // Xét các th?a s? nguyên t? t? 2 d?n sqrt(n)
    for (int i = 2; i <= sqrt(n); i++) {
        while (n % i == 0) {  // Ki?m tra th?a s? nguyên t? i
            n /= i;
            cnt++;  // Ð?m s? l?n xu?t hi?n c?a th?a s? nguyên t?
            if (cnt == k) return i;  // N?u th?a s? th? k xu?t hi?n, tr? v? nó
        }
    }
    
    // Ki?m tra n?u n là m?t s? nguyên t? l?n hon sqrt(n)
    if (n > 1) {
        cnt++;  // Th?a s? nguyên t? cu?i cùng
        if (cnt == k) return n;
    }
    
    return -1;  // Không tìm th?y th?a s? nguyên t? th? k
}

int main() {
    int n, k;
    scanf("%d %d", &n, &k);
    
    int result = thuaso(n, k);
    
    if (result == -1) {
        printf("-1\n");  // Không tìm th?y th?a s? nguyên t? th? k
    } else {
        printf("%d\n", result);  // In ra th?a s? nguyên t? th? k
    }
    
    return 0;
}

