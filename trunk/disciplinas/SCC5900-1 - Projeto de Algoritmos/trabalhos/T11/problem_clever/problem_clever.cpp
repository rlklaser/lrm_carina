/*problem Clever Naming*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>
#include <list>

using namespace std;

#define MAX_ITEMS 30

int correlation[MAX_ITEMS][MAX_ITEMS];

struct item {
    int num;
    list<string>* names;
    //char current[1000];
    int selected;
};

struct ordered_item
{
    int num;
    char name[1000];
};

struct item         items[MAX_ITEMS];
struct ordered_item ordered_items[MAX_ITEMS];

char to_upper(char c)
{
    if(c>='a' && c<='z') c-='a'-'A';
    return c;
}

char to_lower(char c)
{
    if(c>='A' && c<='Z') c-='A'-'a';
    return c;
}

void add_string(int ndx, char* str)
{
    int n = 0;
    char c;
    for(;;) {
        c=str[n];
        if(c==0) break;
        if(n==0)
            c = to_upper(c);
        else
            c = to_lower(c);
        str[n]=c;
        n++;
    }
    string s(str, n);
    items[ndx].names->push_back(s);
}

void clear_items()
{
    int i;
    for(i=0; i<MAX_ITEMS; i++)
    {
        if(items[i].names!=0) {
            items[i].names->clear();
            delete items[i].names;
            items[i].names = 0;
        }
    }

}

int max_correlation(int id)
{
    int i;
    for(i=0; i<MAX_ITEMS; i++) {
        if(correlation[id][i]==1) break;
    }
    return i;
}

void clear_correlation(int id)
{
    int j;
    for(j=0; j<MAX_ITEMS; j++) {
        correlation[j][id] = 0;
    }
}

void clever_name(int qtd)
{
    int i, id;
    int ordered=0;
    int owner;
    string s;
    int my_max, owner_max;

    struct ordered_item* ordered_item;

    if(qtd==1) {
        cout << items[0].names->front() << endl;
    }
    else {
        while(ordered!=qtd) {
            for(i=0; i<qtd; i++) {

                if(items[i].names->size()>0 && items[i].selected==0) {

                    id = -1;

                    while(items[i].names->size()>0) {
                        s = items[i].names->front();
                        items[i].names->pop_front();
                        id = (int)s.c_str()[0]-'A';
                        ordered_item = &ordered_items[id];
                        owner = ordered_item->num-1;

                        /*se nao tem anterior, associa*/
                        if(owner==-1) break;
                        /*se ja tem marcado, verifica se tera chance de troca*/
                        if(items[owner].names->size()!=0) {
                            correlation[i][id] = 0; /*elimina correlacao*/
                            my_max = max_correlation(i);
                            owner_max = max_correlation(owner);
                            /*o anterior tem chance de trocar por um menos pior que o meu proximo*/
                            if(my_max>=owner_max) {
                                ordered--;
                                items[owner].selected = 0;
                                break;
                            }
                        }
                    }

                    /*acabou a lista, fica com o nome, marca como escolhido*/
                    if(items[i].names->size()==0) {
                        clear_correlation(id);
                    }
                    ordered++;
                    items[i].selected = 1;

                    ordered_items[id].num=i+1;
                    sprintf(ordered_items[id].name, "%s", s.c_str());
                    correlation[i][id] = 0;
                }
            }
        }

        id = 0;
        for(i=0; i<MAX_ITEMS; i++) {
            if(ordered_items[i].name[0]!=0) {
                /*printf("%d %s\n", ordered_items[i].num, ordered_items[i].name);*/
                printf("%s\n", ordered_items[i].name);
                id++;
            }
            if(id==qtd) break;
        }

        if(id!=qtd) {
            delete items[0].names;
            cout << items[0].names;
        }
    }
}

/*trecho de codigo extraido do site cplusplus.com*/
bool compare_nocase (string first, string second)
{
  unsigned int i=0;
  while ( (i<first.length()) && (i<second.length()) )
  {
    if (tolower(first[i])<tolower(second[i])) return true;
    else if (tolower(first[i])>tolower(second[i])) return false;
    ++i;
  }
  if (first.length()<second.length()) return true;
  else return false;
}

int main()
{
#ifdef REDIR
    freopen("input.txt","r",stdin);
#endif
    int cases, n, names, name, p, l, j;
    char line[1000];

    scanf("%d", &cases);
    for(n=0; n<cases; n++) {
        printf("Case #%d:\n", n+1);

        memset(items, 0, sizeof(items));
        memset(ordered_items, 0, sizeof(ordered_items));
        memset(correlation, 0, sizeof(correlation));

        scanf("%d", &p);
        for(l=0; l<p; l++) {
            items[l].names=new list<string>();
            scanf("%d", &names);
            for(name=0; name<names; name++) {

                memset(line, 0, sizeof(line));
                scanf("%s", line);

                j = to_upper(line[0])-'A';
                add_string(l, line);
                correlation[l][j] = 1;
            }
            items[l].names->sort(compare_nocase);
        }
        clever_name(p);
        clear_items();
    }
    return 0;
}
