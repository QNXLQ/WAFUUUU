#ifndef _MATRIZ_H_
#define _MATRIZ_H_

#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

namespace matrix
{
    class matriz
    {
        private:
            std::size_t _Row, _Col;
        public:
            double **_Matrix;
            matriz():_Matrix(nullptr), _Row(0), _Col(0){};
            matriz(std::size_t row, std::size_t col);
            matriz(std::size_t row, std::size_t col, const double init);
            matriz(const matriz& m);
            ~matriz();

            double& operator()(std::size_t i, std::size_t j){return _Matrix[i][j];}
            const double operator()(std::size_t i, std::size_t j)const{return _Matrix[i][j];}
            matriz operator=(matriz M);
            matriz operator+(matriz M);
            matriz operator-(matriz M);
            matriz multiply(const matriz &N);
            matriz multiply(matriz &N);
            matriz operator*(matriz M);
            matriz operator*(const double a);
            friend matriz operator*(const double a, matriz M);
            matriz operator/(matriz &M);
            

            matriz trans();
            double detA();
            bool inv(matriz &A);
            double trace();
            bool LSM(matriz &B, matriz &x);
            double* size();
            void PrintM();
    };

    matriz Identity(int n);
    matriz zero(int row, int col);
    matriz Rodrigues(std::vector<double> &w);
    matriz inv_Rodrigues(matriz M);
    matriz LSM(matriz A, matriz B);
    matriz inv(matriz A);
    matriz fake_inv(matriz A);
    matriz mpow(matriz M, int n);
    matriz mexp(matriz M, int n);
}

#endif