#include <stdio.h>
#include <math.h>

int nguyento( long long n ){ 
	int cnt =0; // dem so nguyen to
	
	if ( n ==1 || n ==0) return 0; // 0 1 k phai so nguyen to
	
	for ( int i =2 ; i <= sqrt (n); i ++ ){
		int mu =0; // dem so mu
		while ( n % i == 0){ // check so trung nhau
			mu++;
			n /= i;
		}
		if ( mu >=2 ) return 0; // neu mu >= 2 la so trung => 0 la so sphenic
		if ( mu ==1 ) cnt++;
	}
	if ( n >1 ) cnt++; // so finally luon luon mu 1 => tang cnt
	if ( cnt ==3 ) return 1;
	else return 0;
}

int main (){
	long long n;
	scanf ("%lld", &n);
	if ( nguyento (n) ==1 ) printf ("1");
	else printf ("0");
	return 0;
}
