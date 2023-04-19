#include <iostream>
#include <iterator>
#include <string>
#include <regex>

using namespace std;

void test(regex expr)
{
   string input;   

   do
   {
      cout << "Input: ";
      cin >> input;         
   } while (regex_match(input, expr) == false);

   cout << "Input ok!" << endl;
}


int main()
{
   string s1 =
      "Alan Mathison Turing war ein britischer Logiker, Mathematiker," \
      "Kryptoanalytiker und Informatiker." \
      "Er gilt heute als einer der einflussreichsten Theoretiker der" \
      "frühen Computerentwicklung und Informatik." \
      "Turing schuf einen großen Teil der theoretischen Grundlagen für" \
      "die moderne Informations- und Computertechnologie. Als richtungsweisend" \
      "erwiesen sich auch seine Beiträge zur theoretischen Biologie.";

   regex r1("computer");
   if (regex_search(s1, r1) == true) {
      std::cout << "Text contains the phrase 'computer'\n";
   }
   else {
      std::cout << "Text does not contain the phrase 'computer'\n";
   }

   regex r2("mathematiker", std::regex_constants::icase);
   if (regex_search(s1, r2) == true) {
      std::cout << "Text contains the phrase 'mathematiker'\n";
   }
   else {
      std::cout << "Text does not contain the phrase 'mathematiker'\n";
   }

   string input;
   while (true)
   {   
      /*
      regex r3("[[:digit:]]");
      cout << "\n1. Please enter a single digit, e.g. 5" << endl;
      test(r3);
            
      regex r4("[[:digit:]]+");
      cout << "\n2. Please enter a single or multiple digits (without a sign), e.g. 559" << endl;
      test(r4);

      regex r5("-?[[:digit:]]+");
      cout << "\n3. Please enter a single or multiple digits (with or without a minus sign -), e.g. -559" << endl;
      test(r5);

      regex r6("\\+?[[:digit:]]+");
      cout << "\n4. Please enter a single or multiple digits (with or without a plus sign +), e.g. +559" << endl;
      test(r6);

      regex r7("(\\+|-)?[[:digit:]]+");
      cout << "\n5. Please enter a single or multiple digits (with or without a minus - or plus sign +), e.g. -903" << endl;
      test(r7);

      regex r8("((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?");
      cout << "\n6. Please enter a real number (with or without a minus - or plus sign +), e.g. -3.14159" << endl;
      test(r8);
      */
      
      regex r9("[[:alnum:]]+([[:alnum:]]|\\.)*@[[:alnum:]]+\\.[[:alnum:]]+");
      cout << "\n7.Enter an email address:" << endl;
      test(r9);
      
   }

   
   string str;
   cin >> str;
}