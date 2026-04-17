#include <stdio.h>
#include <math.h>

long long ucln ( long long a, long long b ){
	while ( b != 0 ){
		 int r = a % b ;
		 a = b;
		 b = r;
	}
	return a;
}
long long bcnn ( long long a , long long b){
	return a/ ucln(a, b) * b;
}

int main (){
	long long a, b ; 
	scanf ( "%lld %lld", &a, &b);
	printf ("%lld %lld", ucln ( a, b) , bcnn (a,b ) );
	
	return 0;
}
