#include<stdio.h>
#include<stdlib.h>
#include<windows.h>

int main()
{
    POINT punkt;

    while(1)
    {
        GetCursorPos(&punkt);

        printf("%d",punkt.x);
        printf("\n%d", punkt.y);
        system("cls");
    }
}
