#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#define MAX_WORD_SIZE 16
#define MAX_DICT_SIZE 1000
#define MAX_LETTERS 26
#define MAX_LINE 81
#define POS_FORM_LETTER(X) ((int)('z'-X))
#define LETTER_FROM_POS(X) ((char)('z'-X))
#define MAX_MATCHES 500

struct dict_word
{
    int size;
    char value[MAX_WORD_SIZE];
};

int dist_table[MAX_LETTERS+1][MAX_WORD_SIZE+1];
int dist_table_sentence[MAX_LETTERS+1][MAX_WORD_SIZE+1];
int dictionary_size;
struct dict_word dictionary[MAX_DICT_SIZE];
int matches[MAX_LETTERS][MAX_LETTERS];
char bijection[MAX_LETTERS];
struct sentence_word {
    struct dict_word word;
    int matches[MAX_MATCHES];
};


struct sentence_word sentence[MAX_LINE];
int sentence_words;


void print_matches()
{
    int i, j;
    char c[2];
    c[1]=0;
    printf("  ");
    for(i=0;i<MAX_LETTERS;i++) {
        c[0] = (char)('z'-i);
        printf("%s ", c);
    }
    printf("\n");
    for(i=0;i<MAX_LETTERS;i++) {
        c[0] = (char)('z'-i);
        printf("%s ", c);
        for(j=0;j<MAX_LETTERS;j++) {
            printf("%d ", matches[i][j]);
        }
        printf("\n");
    }
    printf("\n");
}

void print_dist_table(int tbl)
{
    int i, j;
    char c[2];
    //imprime tabela probabilidade
    c[1]=0;
    for(i=0;i<MAX_LETTERS+1;i++) {
        if(i==MAX_LETTERS) {
            printf("T ");
        }
        else {
            c[0] = (char)('z'-i);
            printf("%s ", c);
        }
        for(j=0;j<MAX_WORD_SIZE;j++) {
            if(tbl==0) {
                printf("%d ", dist_table[i][j]);
            }
            else {
                printf("%d ", dist_table_sentence[i][j]);
            }
        }
        if(tbl==0) {
            printf("%d\n", dist_table[i][MAX_WORD_SIZE]);
        }
        else {
            printf("%d\n", dist_table_sentence[i][MAX_WORD_SIZE]);
        }
    }
    printf("\n");

}

int main()
{

    int i, j, k, p, len;
    char l;

    char line[MAX_LINE];

    memset(dictionary, 0, sizeof(dictionary));
    memset(dist_table, 0, sizeof(dist_table));

    FILE* f = freopen("input.txt","r",stdin);

    fscanf(f, "%d", &dictionary_size);

    for(i=0; i<dictionary_size; i++) {

        fscanf(f, "%s", &(dictionary[i].value[0]));
        for(j=0; j<MAX_WORD_SIZE; j++) {
            l = dictionary[i].value[j];
            if(l==0) {
                dictionary[i].size=j;
                break;
            }
            else {

                p=POS_FORM_LETTER(l);
                dist_table[p][j]++; //acumula na posicao
                dist_table[MAX_LETTERS][j]++; //acumula na coluna
                dist_table[p][MAX_WORD_SIZE]++; //acumula na linha

            }

        }
    }

    print_dist_table(0);

    char* ret=1;
    while(ret) {
        memset(line, 0, sizeof(line));
        ret = fgets(line, MAX_LINE, f);
        if(line[0]=='\n' || ret==0) continue; //skip

        memset(bijection, '*', sizeof(bijection));//limpa a tabela de equiv.
        memset(sentence, 0, sizeof(sentence));
        memset(dist_table_sentence, 0, sizeof(dist_table_sentence));
        sentence_words=0;
        k=0;
        len=0;
        //tokenize
        for(i=0; i<MAX_LINE; i++) {
            l = line[i];
            if(l=='\n' || l==' ') {
                sentence[k].word.size=len;
                if(len==0) break;
                len=0;
                k++;
                sentence_words++;
                if(l=='\n') {
                    break; //finaliza
                }
            }
            else {
                sentence[k].word.value[len]=l;

                //dist frase
                p=POS_FORM_LETTER(l);
                dist_table_sentence[p][len]++; //acumula na posicao
                dist_table_sentence[MAX_LETTERS][len]++; //acumula na coluna
                dist_table_sentence[p][MAX_WORD_SIZE]++; //acumula na linha

                len++;
            }
        }

        for(k=0; k<MAX_LETTERS;k++) {
            for(i=0; i<MAX_LETTERS; i++) {
                for(j=0; j<MAX_WORD_SIZE; j++) {
                    if((dist_table_sentence[k][j]==0&&dist_table[i][j]==0) || (dist_table_sentence[k][j]!=0&&dist_table[i][j]!=0)) {
                        //match
                        matches[k][i]=1;
                    }
                    else {
                        matches[k][i]=0;
                        break;
                    }
                }
            }
        }

        for(i=0;i<MAX_LETTERS;i++) {
            if(matches[i][i]==1) {
                bijection[i]=(char)('z'-i);
            }
        }

        print_dist_table(1);

        print_matches();
    }



    return 0;
}
