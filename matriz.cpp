#include "matriz.h"

matrix::matriz::matriz(std::size_t row, std::size_t col)
{
    if (!row || !col)
        return;
    _Matrix = (double **)std::malloc(col * sizeof(double*));
    double **p = _Matrix, **end = _Matrix + col;
    while (p != end)
        *(p++) = (double*)std::malloc(row * sizeof(double));
    _Row = row;
    _Col = col;
}

matrix::matriz::matriz(std::size_t row, std::size_t col, const double init)
{
    if (!row || !col)
        return;
    _Matrix = (double**)std::malloc(col * sizeof(double*));
    double **pr = _Matrix, **endr = _Matrix + col, *p, *end;
    while (pr != endr)
    {
        p = *(pr++) = (double*)std::malloc(row * sizeof(double));
        end = p + row;
        while (p != end)
            *(p++) = init;
    }
    _Row = row;
    _Col = col;
}

matrix::matriz::matriz(const matriz& m)
{
    _Row =  m._Row;
    _Col = m._Col;
    _Matrix = (double**)std::malloc(_Col * sizeof(double*));
    double **pbr = m._Matrix, **endbr = m._Matrix + _Col, *pb, *endb,
                    **par = _Matrix, **endar = _Matrix + _Col, *pa, *enda;
    while(par != endar)
    {
        pa = (*par++) = (double*)std::malloc(_Row * sizeof(double));
        enda = pa + _Row;
        pb = *(pbr++);
        endb = pb + _Row;
        while (pa != enda)
            *(pa++) = *(pb++);
    }
}

matrix::matriz::~matriz()
{
    if(!_Matrix)
        return;
    double **p = _Matrix, **end = _Matrix + _Col;
    while (p != end)
        free(*(p++));
    _Col = _Row = 0;
    free(_Matrix);
}

matrix::matriz matrix::matriz::operator=(matriz M)
{
    if(M._Matrix)
    {
        double **p = _Matrix, **end = _Matrix + _Col;
        while(p != end)
            free(*(p++));
        free(_Matrix);
        _Row = M._Row;
        _Col = M._Col;
        _Matrix = M._Matrix;
    }
    return *this;
}

matrix::matriz matrix::matriz::operator+(matriz M)
{
    if (_Row != M._Row || _Col != M._Col)
    {
        std::cout << "Prohibiden add two matrix have different size!" << std::endl;
        return;
    }

    for (std::size_t i = 0; i < _Row; i++)
        for (std::size_t j = 0; j < _Col; j++)
            _Matrix[i][j] += M(i, j);
    return (*this);
}

matrix::matriz matrix::matriz::operator-(matriz M)
{
    if (_Row != M._Row || _Col != M._Col)
    {
        std::cout << "Prohibiden add two matrix have different size!" << std::endl;
        return;
    }

    for (std::size_t i = 0; i < _Row; i++)
        for (std::size_t j = 0; j < _Col; j++)
            _Matrix[i][j] -= M(i, j);
    return (*this);
}

matrix::matriz matrix::matriz::multiply(const matriz &N)
{
    if (_Col != N._Row)
    {
        std::cout << "Check matrix size for multiply!" << std::endl;
        return *this;
    }
    matriz tmp(_Row, _Col, 0);
    std::size_t i = 0, j, k;
    double **pr, *p;
    while (i < _Row)
    {
        j = 0;
        pr = N._Matrix;
        while (j < N._Col)
        {
            k = 0;
            p = *(pr++);
            while( k < _Col)
            {
                tmp(i, j) += (*this)(k, i) * (*(p++));
                k++;
            }
            j++;
        }
        i++;
    }
    return tmp;
}

matrix::matriz matrix::matriz::multiply(matriz &N)
{
    return this->multiply((const matriz) N);
}

matrix::matriz matrix::matriz::operator*(matrix::matriz M)
{
    return (*this).multiply(M);
}

matrix::matriz matrix::matriz::operator*(const double a)
{
    for (std::size_t i = 0; i< _Row; i++)
        for(std::size_t j = 0; j < _Col; j++)
            (*this)(i, j) *= a;
    return (*this);
}

/**
 * @brief The matrix dont have divide, but here A/B it means A * inv(B)
 * 
 * @param M 
 * @return matriz 
 */
matrix::matriz matrix::matriz::operator/(matriz &M)
{
    matriz tmp(M._Row, M._Col, 0);
    if(M.inv(tmp))
        return (*this) * tmp;
    return (*this);
}

/**
 * @brief  Get AT which is the transposition of A
 * 
 * @return matriz 
 */
matrix::matriz matrix::matriz::trans()
{
    matriz res(_Col, _Row, 0);
    for(std::size_t i; i < _Col; i++)
        for (std::size_t j; j <_Row; j++)
            res._Matrix[j][i] = _Matrix[i][j];
    return res;
}

/**
 * @brief Calculate the det of the matrix
 * 
 * @return double 
 */
double matrix::matriz::detA()
{
    if(_Row == 1)
        return _Matrix[0][0];
    double ans;
    matriz tmp(_Row - 1, _Col - 1, 0);
    for ( std::size_t i = 0; i < _Row; i++)
    {
        for (std::size_t j = 0; j < _Row-1; j++)
            for (std::size_t k = 0; k < _Row-1; k++)
                tmp._Matrix[j][k] = _Matrix[j+1][(k>=i)?k+1:k];
        double t = tmp.detA();
        if (i%2 == 0)
            ans += (*this)(0, i) * t;
        else
            ans -= (*this)(0, i)  * t;
    }
    return ans;
}

/**
 * @brief Inverse the matrix
 * 
 * @param A 
 * @return true 
 * @return false 
 */
bool matrix::matriz::inv(matriz &A)
{
    double flag = detA();
    if(_Row != _Col || 1e-10 > std::fabs(flag))
    {
        std::cout << "Singular matrix! No exist inverse!" << std::endl;
        return false;
    }

    if (_Row == 1)
    {
        A = (*this);
        return true;
    }

    matriz tmp(_Row-1, _Col-1,0);
    for(std::size_t i = 0; i < _Row; i++)
    {
        for(std::size_t j = 0; j < _Row; j++)
        {
            for(std::size_t k = 0; k < _Row-1; k++)
                for(std::size_t t = 0; t < _Row-1; t++)
                    tmp._Matrix[k][t] = _Matrix[k>=i?k+1:k][t>=j?t+1:t];
            A(j, i) = tmp.detA();
            if ((i+j)%2 == 1)
                A(j, i) = -A(j, i);
        }
    }

    for (std::size_t i = 0; i < _Row; i++)
        for (std::size_t j = 0; j < _Row; j++)
            A(j, i) = A(j, i)/flag;
    return true;    
}

/**
 * @brief Calculate the trace of a square matrix
 * 
 * @return double 
 */
double matrix::matriz::trace()
{
    if (_Row != _Col)
    {
        std::cout << "Only Square matrix have trace!" << std::endl;
        return;
    }

    double ans = 0;
    for(std::size_t i = 0; i < _Row; i++)
        ans += (*this)(i, i);
    return ans;
}

/**
 * @brief 
 * This function is Least Mean Square, Ax = B, x = (ATA)^(-1)ATB
 * @param B 
 * @param x 
 * @return true 
 * @return false if ATA is singular
 */
bool matrix::matriz::LSM(matriz &B, matriz &x)
{
    matriz at = (*this).trans();
    matriz ata = at.multiply(*this);
    matriz invata(ata._Col, ata._Row, 0);
    if(!ata.inv(invata))
        return false;
    else
        x = invata.multiply(at).multiply(B);
    return true;
}

double* matrix::matriz::size()
{
    double s[2] ={ _Row, _Col};
    return s;
}

void matrix::matriz::PrintM()
{
    std::cout << "Row = " << _Row << "  and Col = " << _Col << std::endl;
    for (std::size_t i = 0; i < _Row; i++)
        {
            for (std::size_t j =0; j < _Col; j++)
                std::cout << _Matrix[j][i] << "  ";
            std::cout << std::endl;
        }
}

/*Functions in namespace but  class matriz*/

matrix::matriz matrix::Identity(int n)
{
    matrix::matriz tmp(n, n, 0);
    for (std::size_t i = 0; i < n; i++)
        tmp._Matrix[i][i] = 1;
    return tmp;
}

matrix::matriz matrix::zero(int row, int col)
{
    matrix::matriz tmp(row, col, 0);
    return tmp;
}

/**
 * @brief  The Rodrigues Formula use to convert Lie Algebra to Lie Group
 * 
 * @param w is a vector of Lie Algebra, shoule be 1x3 or 1x6, if 1x3 its in so(3), if 1x6 its in se(3)
 * @return matrix::matriz if w is 1x3, then the result is in SO(3); or in SE(3)
 */
matrix::matriz matrix::Rodrigues(std::vector<double> &w)
{
    if (w.size() != 3 && w.size() != 6)
    {
        std::cout << "The size of vector should be equal to 3 or 6!" << std::endl;
        return;
    }
    matrix::matriz a(1, 3, 0);
    double theta = 0;
    if (w.size()==3)
    {
        theta = sqrt(std::pow(w[0], 2) + std::pow(w[1], 2) + std::pow(w[2],2));
        a(0, 0) = w[0]/theta; a(0, 1) = w[1]/theta; a(0, 2) = w[2]/theta;
    }
    else
    {
        theta = sqrt(std::pow(w[3], 2) + std::pow(w[4], 2) + std::pow(w[5],2));
        a(0, 0) = w[3]/theta; a(0, 1) = w[4]/theta; a(0, 2) = w[5]/theta;
    }
    
    matrix::matriz skew(3, 3, 0);
    skew(0, 1)= -a(0, 2); skew(0, 2) = a(0, 1);
    skew(1, 0) = a(0, 2);  skew(1, 2) = -a(0, 0);
    skew(2, 0) = -a(0, 1); skew(2, 1)=a(0, 0); 

    matrix::matriz I = matrix::Identity(3);
    matrix::matriz R = cos(theta) * I + (1-cos(theta)) * a * a.trans() + sin(theta) * skew;
    if (w.size() == 3)
        return R;

    matrix::matriz J = (sin(theta)/theta) * I + (1- sin(theta)/theta) * a * a.trans() + ((1-cos(theta))/theta) * skew;
    matrix::matriz p(1, 3, 0);
    p(0, 0) = w[0]; p(0, 1) = w[1]; p(0, 2) = w[2];
    matrix::matriz Jp = J * p;
    matrix::matriz V(4, 4, 0);
    V(0, 0) = R(0, 0); V(0, 1) = R(0, 1); V(0, 2) = R(0, 2); V(0, 3) = Jp(0, 0);
    V(1, 0) = R(1, 0); V(1, 1) = R(1, 1); V(1, 2) = R(1, 2); V(1, 3) = Jp(0, 1);
    V(2, 0) = R(2, 0); V(2, 1) = R(2, 1); V(2, 2) = R(2, 2); V(2, 3) = Jp(0, 2);
                                                                                                      V(3, 3) = 1;
    return V;
}

/**
 * @brief  This function is use to convert Lie Group to Lie Algebra
 * 
 * @param M is a matrix in Lie Group, should be 3x3 (SO(3)) or 4x4 (SE(3))
 * @return matrix::matriz 
 */
matrix::matriz matrix::inv_Rodrigues(matrix::matriz M)
{
    double* M_size = M.size();
    if (M_size[0] ==3)
    {
        matrix::matriz a = zero(1, 3);
        double tr = M.trace();
        double tmp = (tr-1)/2;
        double theta = acos(tmp);
        a(0,0) = sqrt((M(1, 1) - tmp)/(1 - tmp));
        a(0,1) = sqrt((M(2, 2) - tmp)/(1 - tmp));
        a(0,2) = sqrt((M(3, 3) - tmp)/(1 - tmp));
        return a * theta;
    }

    if (M_size[0] == 4)
    {
        matrix::matriz v = zero(1, 6);
        double tr = M.trace() - 1;
        double tmp = (tr - 1)/2;
        double theta = acos(tmp);
        matrix::matriz a = zero(1, 3);
        a(0,0) = v(0, 3) = sqrt((M(1, 1) - tmp)/(1 - tmp)) * theta;
        a(0,1) = v(0, 4) = sqrt((M(2, 2) - tmp)/(1 - tmp)) * theta;
        a(0,2) = v(0, 5) = sqrt((M(3, 3) - tmp)/(1 - tmp)) * theta;
        matrix::matriz skew(3, 3, 0);
        skew(0, 1)= -a(0, 2); skew(0, 2) = a(0, 1);
        skew(1, 0) = a(0, 2);  skew(1, 2) = -a(0, 0);
        skew(2, 0) = -a(0, 1); skew(2, 1)=a(0, 0); 
        matrix::matriz invJ = (theta/2*tan( M_PI_2-theta)) * Identity(3) + (1-theta/2*tan( M_PI_2-theta)) * a * a.trans() + theta/2 * skew;
        matrix::matriz T(3, 1, 0);
        T(0,0) = M(0, 3); T(1,0) = M(1, 3); T(2, 0) = M(2, 3);
        matrix::matriz p = invJ * T;
        v(0, 0) = p(0, 0); v(1, 0) = p(1, 0); v(2, 0) = p(2, 0);
        return v;
    }

    return;
}

/**
 * @brief  Least Square Method
 *  Ax = B, x = inv((AT)A)(AT)B
 * @param A mxn
 * @param B mx1
 * @return matrix::matriz output is x which is nx1
 */
matrix::matriz matrix::LSM(matrix::matriz A, matrix::matriz B)
{
    matrix::matriz ATA = A.trans() * A;
    if (1e-10 > std::fabs(ATA.detA()))
        return;
    matrix::matriz x = inv(ATA) * A.trans() * B;
}

matrix::matriz matrix::inv(matrix::matriz M)
{
    matrix::matriz tmp;
    if (M.inv(tmp))
        return tmp;
    return;
}

/*
matrix::matriz matrix::fake_inv(matrix::matriz A)
{
    double* A_size = A.size();
    matrix::matriz res;
    if (A_size[0] > A_size[1])      //Full rank in row, fake inverse is left inverse
    {
        res = inv(A.trans() * A) * A.trans();
    }
    else if (A_size[0] < A_size[1])     //Full rank in col, fake inverse is right inverse
    {
        res = A.trans() * inv(A.trans() * A);
    }
    else
    {

    }

    return res;
}
*/
/**
 * @brief  Calculate power of matrix
 * 
 * @param M mxm matrix
 * @param n times of power
 * @return matrix::matriz 
 */
matrix::matriz matrix::mpow(matrix::matriz M, int n)
{
     double* M_size = M.size();
    if (M_size[0] != M_size[1])
    {
        std::cout << "The matrix should be a square matrix!" << std::endl;
        return;
    }
    if (n == 0)
        return Identity(M_size[0]);
    matrix::matriz tmp = M;
    for (std::size_t i = 0; i < n; i++)
        tmp = tmp * M;
    return tmp;
}

/**
 * @brief Calculate e^(At), where A is a matrix, t is times of power
 * with Taylor Formula
 * @param M mxm matrix
 * @param n times of power
 * @return matrix::matriz 
 */
matrix::matriz matrix::mexp(matrix::matriz M, int n)
{
    double* M_size = M.size();
    if (M_size[0] != M_size[1])
    {
        std::cout << "The matrix should be a square matrix!" << std::endl;
        return;
    }
    matrix::matriz e = Identity(M_size[0]);
    double coef = 1;
    for (std::size_t i = 1; i < n; i++)
    {
        e = e + 1/coef * matrix::mpow(M, i);
        coef *= (i+1);
    }
    return e;
}