#include "Matrix.h"

Matrix::Matrix(int row, int col, float* arr){
    this->row = row;
    this->col = col;
    this->elem = arr;
}

float& Matrix::at(int row, int col){
    return elem[getIdx(row, col)];
}

int Matrix::getRow(){
    return this->row;
}

int Matrix::getCol(){
    return this->col;
}

int Matrix::getIdx(int row, int col){
    return (row*this->col)+col;
}

Matrix::~Matrix(){

}

void matTranspose(Matrix a, Matrix result){
    int row_a = a.getRow(), col_a = a.getCol();
    for(int i=0;i<row_a;i++){
        for(int j=0;j<col_a;j++){
            result.at(j, i) = a.at(i, j);
        }
    }
}

void matPlus(Matrix a, Matrix b, Matrix result){
    int row_a = a.getRow(), col_a = a.getCol();
    int row_b = b.getRow(), col_b = b.getCol();
    for(int i=0;i<row_a;i++){
        for(int j=0;j<col_a;j++){
            result.at(i, j) = a.at(i, j) + b.at(i, j);
        }
    }
}

void matSub(Matrix a, Matrix b, Matrix result){
    int row_a = a.getRow(), col_a = a.getCol();
    int row_b = b.getRow(), col_b = b.getCol();
    for(int i=0;i<row_a;i++){
        for(int j=0;j<col_a;j++){
            result.at(i, j) = a.at(i, j) - b.at(i, j);
        }
    }
}

void matMul(Matrix a, Matrix b, Matrix result){
    int row_a = a.getRow(), col_a = a.getCol();
    int row_b = b.getRow(), col_b = b.getCol();
    for(int i=0;i<row_a;i++){
        for(int j=0;j<col_b;j++){
            float total = 0;
            for(int k=0;k<col_a;k++){
                total += a.at(i, k) * b.at(k, j);
            }
            result.at(i, j) = total;
        }
    }
}