#include "week5.hpp"
#include <iostream>

// Definition (implementation)
int add_numbers(int a, int b){return a+ b;}

void prompt_user()
{
    std::cout << "Enter a number: ";
    int num{};
    std::cin >> num;

    print_number(num); // ERROR!
}

//==================
void print_number(int number)
{
    if (number < 0)
    {
        std::cout << "Error: Negative numbers not allowed.\n";
        return; // Exit the function immediately
    }
    std::cout << "The number is: " << number << "\n";
}