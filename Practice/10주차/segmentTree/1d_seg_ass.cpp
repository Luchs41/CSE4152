#define _CRT_SECURE_NO_WARNINGS 
#include <cstdio>
#include <memory>
#include <queue>
#include <utility>
#include <cmath>
#include <iostream>

// SegmentTree1D 구현하세요. 직접 스크래치로 전부 구현하셔도 무관합니다.
// 원소 합을 return 해주는 sum 함수와 원소 수정에 대한 modify 함수 구현

using namespace std;

class Node {
public:
    Node(int left, int right) {

        this->left = left;
        this->right = right;
        mid = (left + right) >> 1;
        sum = 0;
        leftNode = NULL;
        rightNode = NULL;

    }
    int left, right, mid, sum;
    Node* leftNode, * rightNode;
};
typedef std::pair<int, int> pairi2; //int 형 두 개체를 단일 개체로 처리 목적

class SegmentTree1D {

public:
    SegmentTree1D(int n) {
        //n개 사이즈 만큼 할당
        this->n = n;
        m = 1;
        int temp = n;
        while (temp) {
            m <<= 1;
            temp >>= 1;
        }

        array = new int[n];
        cum_array = new int[n];
        root = new Node(0, n - 1);

    }

    // 초기화
    void initialize() {
        int sum = 0;
        for (int i = 0;i < n;i++) {
            sum += array[i];
            cum_array[i] = sum;
        }

    }

    int sum(int x, int y) {
    /*
        x에서 y 까지의 합
    */
        return cum_array[y] - cum_array[x - 1];

    }

    void modify(int idx, int num) {
    /*
        idx에 위치한 원소를 num으로 교체
    */
        array[idx] = num;
        initialize();
        return;
    }
    
    

    int n;
    int m;
    int* array;
    int* cum_array;
    Node* root;
};

int main() {

    int n, m;
    FILE* in = fopen("input_assignment1.txt", "r");

    fscanf(in, "%d", &n);
    //cin >> n;
    SegmentTree1D A(n);

    int temp;
    for (int i = 0;i < n;i++) {
        fscanf(in, "%d", &temp);
        //cin >> temp;
        printf("%d ", temp);
        A.array[i] = temp;
    }
    printf("\n");

    A.initialize();

    fscanf(in, "%d", &m);
    //cin >> m;
    for (int i = 0;i < m;i++) {

        fscanf(in, "%d\n", &temp);
        //cin >> temp;
        if (temp == 0) {
            int st, ed;
            fscanf(in, "%d %d", &st, &ed);
            //cin >> st >> ed;
            printf("sum (%d,%d): %d\n", st, ed, A.sum(st, ed));

        }
        else {
            int idx, num;
            fscanf(in, "%d %d", &idx, &num);
            //cin >> idx >> num;
            printf("change %dth elem with %d\n", idx + 1, num);
            A.modify(idx, num);
        }
    }

    return 0;

}