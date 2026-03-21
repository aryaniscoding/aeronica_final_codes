/**
  ******************************************************************************
  * @file    matrix.c
  * @brief   Minimal fixed-size matrix library — implementation
  ******************************************************************************
  */
#include "matrix.h"
#include <string.h>

/* ——————————————————————————————————————————————————————
 *  Zero-initialise
 * —————————————————————————————————————————————————————— */
void mat_zeros(Matrix *m, uint8_t rows, uint8_t cols)
{
    m->rows = rows;
    m->cols = cols;
    memset(m->data, 0, sizeof(m->data));
}

/* ——————————————————————————————————————————————————————
 *  Set from flat row-major array
 * —————————————————————————————————————————————————————— */
void mat_set(Matrix *m, uint8_t rows, uint8_t cols, const float *src)
{
    m->rows = rows;
    m->cols = cols;
    for (uint8_t r = 0; r < rows; r++)
        for (uint8_t c = 0; c < cols; c++)
            m->data[r][c] = src[r * cols + c];
}

/* ——————————————————————————————————————————————————————
 *  Matrix multiply  C = A * B
 * —————————————————————————————————————————————————————— */
void mat_mul(const Matrix *A, const Matrix *B, Matrix *C)
{
    mat_zeros(C, A->rows, B->cols);
    for (uint8_t i = 0; i < A->rows; i++)
        for (uint8_t j = 0; j < B->cols; j++)
            for (uint8_t k = 0; k < A->cols; k++)
                C->data[i][j] += A->data[i][k] * B->data[k][j];
}

/* ——————————————————————————————————————————————————————
 *  Element-wise addition  C = A + B
 * —————————————————————————————————————————————————————— */
void mat_add(const Matrix *A, const Matrix *B, Matrix *C)
{
    C->rows = A->rows;
    C->cols = A->cols;
    for (uint8_t i = 0; i < A->rows; i++)
        for (uint8_t j = 0; j < A->cols; j++)
            C->data[i][j] = A->data[i][j] + B->data[i][j];
}

/* ——————————————————————————————————————————————————————
 *  Element-wise subtraction  C = A - B
 * —————————————————————————————————————————————————————— */
void mat_sub(const Matrix *A, const Matrix *B, Matrix *C)
{
    C->rows = A->rows;
    C->cols = A->cols;
    for (uint8_t i = 0; i < A->rows; i++)
        for (uint8_t j = 0; j < A->cols; j++)
            C->data[i][j] = A->data[i][j] - B->data[i][j];
}

/* ——————————————————————————————————————————————————————
 *  Scalar multiply  B = s * A
 * —————————————————————————————————————————————————————— */
void mat_scale(const Matrix *A, float s, Matrix *B)
{
    B->rows = A->rows;
    B->cols = A->cols;
    for (uint8_t i = 0; i < A->rows; i++)
        for (uint8_t j = 0; j < A->cols; j++)
            B->data[i][j] = A->data[i][j] * s;
}

/* ——————————————————————————————————————————————————————
 *  Matrix–vector multiply  y = A * x
 *  x is a column vector of length A->cols
 *  y is a column vector of length A->rows
 * —————————————————————————————————————————————————————— */
void mat_mul_vec(const Matrix *A, const float *x, float *y)
{
    for (uint8_t i = 0; i < A->rows; i++)
    {
        y[i] = 0.0f;
        for (uint8_t j = 0; j < A->cols; j++)
            y[i] += A->data[i][j] * x[j];
    }
}
