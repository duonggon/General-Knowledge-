#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "strutils.h"

void str_reverse(char *s) {
    int len = strlen(s);
    for (int i = 0; i < len / 2; i++) {
        char temp = s[i];
        s[i] = s[len - i - 1];
        s[len - i - 1] = temp;
    }
}

void str_trim(char *s) {
    int start = 0, end = strlen(s) - 1;
    while (isspace((unsigned char)s[start])) start++;
    while (end >= start && isspace((unsigned char)s[end])) end--;
    int j = 0;
    for (int i = start; i <= end; i++) {
        s[j++] = s[i];
    }
    s[j] = '\0';
}

int str_to_int(const char *s, int *out) {
    if (!s || !*s) return 0;
    int sign = 1;
    long result = 0;
    while (isspace((unsigned char)*s)) s++;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    while (*s) {
        if (!isdigit((unsigned char)*s)) return 0;
        result = result * 10 + (*s - '0');
        s++;
    }
    *out = (int)(result * sign);
    return 1;
}
