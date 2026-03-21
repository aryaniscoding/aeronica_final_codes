/**
  ******************************************************************************
  * @file    matrix.h
  * @brief   Minimal fixed-size matrix library for embedded LQR / MPC
  *          No dynamic allocation — all matrices are stack-allocated via macros
  ******************************************************************************
  */
#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>

/* Maximum dimensions — adjust if your state/control vectors grow */
#define MAT_MAX_ROWS  12
#define MAT_MAX_COLS  12

/**
 * @brief Fixed-capacity matrix stored row-major
 */
typedef struct {
    uint8_t rows;
    uint8_t cols;
    float   data[MAT_MAX_ROWS][MAT_MAX_COLS];
} Matrix;

/* ---- Core operations ---- */

/** @brief Zero-initialise a matrix of given size */
void mat_zeros(Matrix *m, uint8_t rows, uint8_t cols);

/** @brief Set matrix from a flat row-major C array */
void mat_set(Matrix *m, uint8_t rows, uint8_t cols, const float *src);

/** @brief  C = A * B  (result stored in C) */
void mat_mul(const Matrix *A, const Matrix *B, Matrix *C);

/** @brief  C = A + B */
void mat_add(const Matrix *A, const Matrix *B, Matrix *C);

/** @brief  C = A - B */
void mat_sub(const Matrix *A, const Matrix *B, Matrix *C);

/** @brief  B = scalar * A */
void mat_scale(const Matrix *A, float s, Matrix *B);

/** @brief  y = A * x  where x and y are column vectors stored as flat arrays */
void mat_mul_vec(const Matrix *A, const float *x, float *y);

#endif /* MATRIX_H */
