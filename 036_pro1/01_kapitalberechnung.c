#include <stdio.h>

int main()
{
   float cap, cap_goal, rate;
   
   printf("Enter your start capital:");
   scanf("%f", &cap);

   printf("Enter your goal:");
   scanf("%f", &cap_goal);

   printf("Rate:");
   scanf("%f", &rate);

   printf("cap=%f, cap_goal=%f, rate=%f", cap, cap_goal, rate);

   int year = 0;
   while (cap < cap_goal)
   {
      year = year + 1;
      cap = cap + (rate / 100.0f) * cap;
      printf("\n\tYear %d: capital=%f", year, cap);
   }
   printf("\n\nYou have reached your goal after %d years.\n", year);   
}
