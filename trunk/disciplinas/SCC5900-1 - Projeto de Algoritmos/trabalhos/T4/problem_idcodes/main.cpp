#include <stdio.h>
#include <string.h>

void reverse(char* a, char* b)
{
}

void swap(char* a, char* b)
{
    char* c;
    c = a;
    a = b;
    b = c;
}

int nextp(char* first, char* last) {
    if (first == last) return false;
    char* i = first;
    ++i;
    if (i == last) return false;
    i = last;
    --i;

    for(;;) {
        char* ii = i--;
        if (*i <*ii) {
            char* j = last;
            while (!(*i <*--j));
            swap(i, j);
            reverse(ii, last);
            return true;
        }
        if (i == first) {
            reverse(first, last);
            return false;
        }
    }
}

int main () {
  int freq[26];
  char str[80];
  char strcpy[80];
  char c;
  int i, r, j;
  int flag;
#ifdef REDIR
  freopen("input.txt","r",stdin);
#endif

  for(;;)
  {
    flag = 0;
    memset(str, 0, sizeof(str));
    memset(strcpy, 0, sizeof(strcpy));
    memset(freq, 0, sizeof(freq));

    r = scanf("%s", str);
    if(str[0]=='#') break;

    c = str[0];
    i=0;
    while(c!=0) {
       c=str[++i];
    }

    if(nextp(str,str+i))
    {
        printf("%s\n",str);
    }
    else {
        printf("No Successor\n");
    }

  }

  return 0;
}
