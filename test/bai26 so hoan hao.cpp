#include <stdio.h>
#include <math.h>

int nt ( long long n ){ // check so nguyen to
	if ( n < 2 ) return 0 ;
	
	for ( int i = 2 ; i <= sqrt(n) ; i++){
		if ( n % i == 0 ) return 0;
	}
	return 1;
}
long long hh( long long n ){ //dinh li euclid euler

	for ( int p =1 ; p <= 32 ; p++){ // 1=N=9*10^18 => 2^(2p) => p xap xi 32
	
		if ( nt(p) ){ // P so ngto
	
		    long long tmp = (long long ) pow( 2,p ) -1 ;
		    if ( nt(tmp) ){ //2^p -1 la so ngto
		
			    long long num = (long long) pow(2,p-1) *tmp; // => so perfect
			    if ( num == n ) return 1;
		    }
	    }
	}
	return 0;
}

int main (){
	long long n;
	scanf ("%lld", &n);
	
	if ( hh(n) == 1) printf ("YES");
	else printf ("NO");
	
	return 0;
}
