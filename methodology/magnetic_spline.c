#include <stdint.h>

static void contract_spline(int64_t count, const double *coefficients, const int64_t *strides,
                     const int64_t *orders, const int64_t *offsets,
                     const double *weights_x, const double *weights_y,
                     const double *weights_z, double *output) {
    for (int64_t n = 0; n < count; ++n) {
        double sum_x[3] = {0., 0., 0.};
        const int64_t base = offsets[3*n]*strides[0] + offsets[3*n+1]*strides[1] + offsets[3*n+2]*strides[2];
        for (int64_t i = 0; i <= orders[0]; ++i) {
            double sum_y[3] = {0., 0., 0.};
            for (int64_t j = 0; j <= orders[1]; ++j) {
                double sum_z[3] = {0., 0., 0.};
                const double *row = coefficients + base + i*strides[0] + j*strides[1];
                for (int64_t k = 0; k <= orders[2]; ++k) {
                    const double weight = weights_z[n*(orders[2]+1)+k];
                    for (int c = 0; c < 3; ++c) sum_z[c] += row[k*strides[2]+c*strides[3]]*weight;
                }
                const double weight = weights_y[n*(orders[1]+1)+j];
                for (int c = 0; c < 3; ++c) sum_y[c] += sum_z[c]*weight;
            }
            const double weight = weights_x[n*(orders[0]+1)+i];
            for (int c = 0; c < 3; ++c) sum_x[c] += sum_y[c]*weight;
        }
        for (int c = 0; c < 3; ++c) output[3*n+c] = sum_x[c];
    }
}

static int64_t spline_basis(double x, const double *knots, int64_t length, int64_t degree, double *basis) {
    const int64_t count = length-degree-1;
    int64_t low=degree, high=count;
    while (high-low>1) {
        const int64_t middle=(low+high)/2;
        if (x<knots[middle]) high=middle;
        else low=middle;
    }
    double left[degree+1], right[degree+1];
    basis[0]=1.;
    for (int64_t j=1;j<=degree;++j) {
        left[j]=x-knots[low+1-j];
        right[j]=knots[low+j]-x;
        double saved=0.;
        for (int64_t r=0;r<j;++r) {
            const double denominator=right[r+1]+left[j-r];
            const double value=denominator==0. ? 0. : basis[r]/denominator;
            basis[r]=saved+right[r+1]*value;
            saved=left[j-r]*value;
        }
        basis[j]=saved;
    }
    return low-degree;
}

void evaluate_spline(int64_t count, const double *points, const double *coefficients,
                     const int64_t *strides, const int64_t *orders, const int64_t *lengths,
                     const double *knots_x, const double *knots_y, const double *knots_z, double *output) {
    for (int64_t n=0;n<count;++n) {
        double weights_x[orders[0]+1], weights_y[orders[1]+1], weights_z[orders[2]+1];
        int64_t offsets[3];
        offsets[0]=spline_basis(points[3*n],knots_x,lengths[0],orders[0],weights_x);
        offsets[1]=spline_basis(points[3*n+1],knots_y,lengths[1],orders[1],weights_y);
        offsets[2]=spline_basis(points[3*n+2],knots_z,lengths[2],orders[2],weights_z);
        contract_spline(1,coefficients,strides,orders,offsets,weights_x,weights_y,weights_z,output+3*n);
    }
}
