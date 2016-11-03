/*
 * NonLinearRevoluteMeasurementPdf.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014).
 * This implementation can be used to reproduce the results of the paper and to be applied to new research.
 * The implementation allows also to be extended to perceive different information/models or to use additional sources of information.
 * A detail explanation of the method and the system can be found in our paper.
 *
 * If you are using this implementation in your research, please consider citing our work:
 *
@inproceedings{martinmartin_ip_iros_2014,
Title = {Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors},
Author = {Roberto {Mart\'in-Mart\'in} and Oliver Brock},
Booktitle = {Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems},
Pages = {2494-2501},
Year = {2014},
Location = {Chicago, Illinois, USA},
Note = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Url = {http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/martinmartin_ip_iros_2014.pdf},
Projectname = {Interactive Perception}
}
 * If you have questions or suggestions, contact us:
 * roberto.martinmartin@tu-berlin.de
 *
 * Enjoy!
 */

#ifndef NONLINEARREVOLUTEMEASUREMENTPDF_H_
#define NONLINEARREVOLUTEMEASUREMENTPDF_H_

#include <pdf/analyticconditionalgaussian_additivenoise.h>
#include <pdf/gaussian.h>

namespace BFL
{
class NonLinearRevoluteMeasurementPdf : public AnalyticConditionalGaussianAdditiveNoise
{
public:

    /**
     * Constructor
     */
    NonLinearRevoluteMeasurementPdf(const Gaussian& additiveNoise);

    /**
     * Destructor
     */
    virtual ~NonLinearRevoluteMeasurementPdf();

    /**
     * @brief  Get the expected value E[x] of the pdf
     * Get low order statistic (Expected Value) of this AnalyticPdf
     *
     * @return MatrixWrapper::ColumnVector The Expected Value of the Pdf (a ColumnVector with DIMENSION rows)
     */
    virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;

    /**
     * @brief Returns derivative from function to n-th conditional variable
     *
     * @param i Number of the conditional variable to use for partial derivation
     * @return MatrixWrapper::Matrix Partial derivative with respect to conditional
     */
    virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

private:
    mutable MatrixWrapper::Matrix dfx;

};
}

#endif /* NONLINEARREVOLUTEMEASUREMENTPDF_H_ */
