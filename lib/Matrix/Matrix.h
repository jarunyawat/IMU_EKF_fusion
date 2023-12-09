#pragma once

class Matrix{
    public:
        Matrix(int row, int col, float* arr);
        float& at(int row, int col);
        int getRow();
        int getCol();
        ~Matrix();
    private:
        int row=0, col=0;
        float* elem;
        int getIdx(int row, int col);
};

void matTranspose(Matrix a, Matrix result);
void matPlus(Matrix a, Matrix b, Matrix result);
void matSub(Matrix a, Matrix b, Matrix result);
void matMul(Matrix a, Matrix b, Matrix result);