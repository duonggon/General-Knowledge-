#include <stdio.h>

int check ( long long n ){
	
	for ( int i =1 ; i <= n /111 ; i++){
		
		long long tmp = n - 111*i;
		
		if ( tmp % 11 == 0) return 1;
	}
	
	return 0;
} 	

int main (){
	long long n ;
	scanf ("%lld", &n);
	
	if ( check(n) == 1 ) printf ("YES");
	else printf ("NO");
	
	return 0;
}
