#include <stdio.h>

long long demso ( long long n){
	long long cnt=0;
	while ( n != 0 ){
		cnt++;
		n /= 10;
	}
	return cnt;
}

int check ( long long k){
	long long nhan =1;
	
 // neu co 2 chu so khong phai la so dep
    if ( demso(k) < 2) return 0;
    
	for ( int i =1 ; i <= demso(k) -1 ; i++){
		nhan *= 10;
	}
	
	long long first = k / nhan ;
	long long final = k % 10;
	
	if ( first  == 2* final || first * 2 == final ) {
		long long bodaucuoi = (k%nhan) / 10 ;// bo dau va cuoi
		long long conlai = bodaucuoi ;
		long long res = 0;
		
		while ( conlai != 0 ){ // dao nguoc so
			res = res * 10 + conlai % 10;
			conlai /= 10;
		}
		if ( bodaucuoi == res ) return 1;
	}
	return 0;	
}

int main (){
	long long n ;
	scanf ("%lld", &n);
	
	if ( check (n) ==1 ) printf ("1");
	else printf ("0");
	return 0;
}
