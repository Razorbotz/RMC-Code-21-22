#include <iostream>


int main(){

	int x=3;

	std::cout << x << std::endl;

	std::cout << &x << std::endl;

	//int *px;	
	int* px;
	px=&x;

	std::cout << px << std::endl;
	std::cout << &px << std::endl;

 	std::cout << *px << std::endl;	

}
