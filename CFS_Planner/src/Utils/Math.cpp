#include "cfslib/Utils/Math.hpp"
#include "QuadProg++.hh" // delete after qp is redone

#define IKFAST_NO_MAIN
#define IKFAST_HAS_LIBRARY // Add this before including ikfast headers
#define IKFAST_NAMESPACE ikfastgp7
#include "ikfast61gp7.cpp"
#undef IKFAST_NAMESPACE
#define IKFAST_NAMESPACE ikfastgp12
#include "ikfast61gp12.cpp"
#undef IKFAST_NAMESPACE

#undef inverse

namespace cfslib
{
    namespace math
    {

        template Eigen::Matrix<float, Eigen::Dynamic, 1> ToEigen<float>(std::vector<float> data);
        template Eigen::Matrix<double, Eigen::Dynamic, 1> ToEigen<double>(std::vector<double> data);

        /* -------------------------------------------------------------------------- */
        /*                                   Matrix                                   */
        /* -------------------------------------------------------------------------- */
        Eigen::MatrixXd PInv(const Eigen::MatrixXd &M)
        {
            auto nrow = M.rows();
            auto ncol = M.cols();
            Eigen::MatrixXd Minv;

            if (nrow > ncol)
            {
                Minv = ((M.transpose() * M).inverse()) * M.transpose();
            }
            else if (nrow < ncol)
            {
                Minv = M.transpose() * ((M * M.transpose()).inverse());
            }
            else
            {
                Minv = M.inverse();
            }

            return Minv;
        }

        Eigen::MatrixXd EigenVcat(const Eigen::MatrixXd &mat1, const Eigen::MatrixXd &mat2)
        {
            /**
             * [mat1; mat2]
             */

            try
            {
                if (mat1.rows() == 0)
                {
                    return mat2;
                }
                else if (mat2.rows() == 0)
                {
                    return mat1;
                }
                else
                {
                    if (mat1.cols() != mat2.cols())
                    {
                        std::ostringstream ss;
                        ss << ERR_HEADER << "Expected mat1 and mat2 to have same cols(), got ["
                           << mat1.cols() << "], [" << mat2.cols() << "]\n";
                        throw std::runtime_error(ss.str());
                    }
                    Eigen::MatrixXd new_mat(mat1.rows() + mat2.rows(), mat1.cols());
                    new_mat << mat1, mat2;
                    return new_mat;
                }
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::MatrixXd EigenHcat(const Eigen::MatrixXd &mat1, const Eigen::MatrixXd &mat2)
        {
            /**
             * [mat1 mat2]
             */

            try
            {
                if (mat1.cols() == 0)
                {
                    return mat2;
                }
                else if (mat2.cols() == 0)
                {
                    return mat1;
                }
                else
                {
                    if (mat1.rows() != mat2.rows())
                    {
                        std::ostringstream ss;
                        ss << ERR_HEADER << "Expected mat1 and mat2 to have same rows(), got ["
                           << mat1.rows() << "], [" << mat2.rows() << "]\n";
                        throw std::runtime_error(ss.str());
                    }
                    Eigen::MatrixXd new_mat(mat1.rows(), mat1.cols() + mat2.cols());
                    new_mat << mat1, mat2;
                    return new_mat;
                }
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::VectorXd ColWiseFlatten(const Eigen::MatrixXd &mat)
        {
            Eigen::MatrixXd mat_buf;
            if (mat.IsRowMajor)
            {
                mat_buf = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>(mat);
            }
            else
            {
                mat_buf = mat;
            }
            assert(!mat_buf.IsRowMajor);
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>> vec(mat_buf.data(), mat_buf.size());
            return vec;
        }

        Eigen::MatrixXd ColWiseUnflatten(const Eigen::VectorXd &vec, const int &nrow)
        {
            try
            {
                Eigen::VectorXd vec_buf(vec);
                int ncol{(int)vec.size() / nrow};
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> mat(vec_buf.data(), nrow, ncol);
                return mat;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::MatrixXd LinInterp(const Eigen::VectorXd &a, const Eigen::VectorXd &b, const int &T)
        {
            try
            {
                Eigen::MatrixXd mat(a.rows(), T + 1);
                Eigen::VectorXd d = (b - a) / T;
                mat.col(0) = a;
                mat.col(T) = b;
                for (int i = 1; i < T; ++i)
                {
                    mat.col(i) = mat.col(i - 1) + d;
                }
                return mat;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::MatrixXd LinColExpand(const Eigen::MatrixXd &mat, const int &i, const int &n)
        {
            try
            {
                Eigen::MatrixXd new_mat(mat.rows(), mat.cols() + n);
                new_mat << mat.leftCols(i),
                    LinInterp(mat.col(i), mat.col(i + 1), n + 1),
                    mat.rightCols(mat.cols() - i - 2);
                return new_mat;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::MatrixXd BatchCross(const Eigen::Matrix<double, 3, Eigen::Dynamic> &mat, const Eigen::Vector3d &vec)
        {
            try
            {
                if (mat.rows() != 3)
                {
                    std::ostringstream ss;
                    ss << ERR_HEADER << "Expect mat to have shape [3, ?], got [" << mat.rows()
                       << ", " << mat.cols() << "].";
                    throw std::runtime_error(ss.str());
                }

                Eigen::MatrixXd ret(mat.rows(), mat.cols());
                for (uint i = 0; i < mat.cols(); ++i)
                {
                    ret.col(i) = mat.col(i).cross(vec);
                }

                return ret;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::Matrix3d skew(const Eigen::Vector3d &vec)
        {
            Eigen::Matrix3d skew_mat;
            skew_mat << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
            return skew_mat;
        }

        /* -------------------------------------------------------------------------- */
        /*                         QuadProg++ (to be deleted)                         */
        /* -------------------------------------------------------------------------- */

        void SetEigenMatFromQuad(Eigen::VectorXd &emat, const quadprogpp::Vector<double> &qvec)
        {
            int nrow = qvec.size();
            emat.resize(nrow);
            for (int i = 0; i < nrow; ++i)
            {
                emat(i) = qvec[i];
            }
        }

        void SetQuadMatFromEigen(quadprogpp::Matrix<double> &qmat, const Eigen::MatrixXd &emat)
        {
            int nrow = emat.rows(), ncol = emat.cols();
            qmat.resize(nrow, ncol);
            for (int i = 0; i < nrow; ++i)
            {
                for (int j = 0; j < ncol; ++j)
                {
                    qmat[i][j] = emat(i, j);
                }
            }
        }

        void SetQuadVecFromEigen(quadprogpp::Vector<double> &qvec, const Eigen::MatrixXd &evec)
        {
            int nrow = evec.rows(), ncol = evec.cols();
            assert(ncol == 1);
            qvec.resize(nrow);
            for (int i = 0; i < nrow; ++i)
            {
                qvec[i] = evec(i, 0);
            }
        }

        /* -------------------------------------------------------------------------- */
        /*                               Transformation                               */
        /* -------------------------------------------------------------------------- */

        template <typename T>
        Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data)
        {
            Eigen::Matrix<T, Eigen::Dynamic, 1> data_eigen;
            size_t n = data.size();
            data_eigen.resize(n, 1);
            for (size_t i = 0; i < n; ++i)
            {
                data_eigen(i) = data.at(i);
            }
            return data_eigen;
        }

        void TransMatToPoseEulerRad(const Eigen::Matrix4d &mat, Vector6d &euler)
        {
            euler.head(3) = mat.block<3, 1>(0, 3);
            euler.tail(3) = mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0); // Euler ZYX
        }

        void TransMatToPoseEulerDeg(const Eigen::Matrix4d &mat, Vector6d &euler)
        {
            TransMatToPoseEulerRad(mat, euler);
            euler(3) = RAD2DEG(euler(3));
            euler(4) = RAD2DEG(euler(4));
            euler(5) = RAD2DEG(euler(5));
        }

        void TransMatToPoseAngRad(const Eigen::Matrix4d &mat, Vector6d &p)
        {
            p.head(3) = mat.block<3, 1>(0, 3);

            Eigen::AngleAxisd ang{mat.block<3, 3>(0, 0)};

            p(3) = ang.axis()(0) * ang.angle();
            p(4) = ang.axis()(1) * ang.angle();
            p(5) = ang.axis()(2) * ang.angle();
        }

        void TransMatToPoseAngDeg(const Eigen::Matrix4d &mat, Vector6d &p)
        {
            TransMatToPoseAngRad(mat, p);
            p(5) = RAD2DEG(p(5));
            p(4) = RAD2DEG(p(4));
            p(3) = RAD2DEG(p(3));
        }

        void PoseAngRadToTransMat(const Vector6d &p, Eigen::Matrix4d &mat)
        {
            mat.block(0, 3, 3, 1) = p.head(3);
            mat(3, 0) = 0;
            mat(3, 1) = 0;
            mat(3, 2) = 0;
            mat(3, 3) = 1;

            Eigen::Matrix3d rot_mat{Eigen::AngleAxisd{p.tail(3).norm(), p.tail(3).normalized()}};
            mat.block(0, 0, 3, 3) = rot_mat;
        }

        void PoseEulerRadToTransMat(const Vector6d &euler, Eigen::Matrix4d &mat)
        {
            mat.block<3, 1>(0, 3) = euler.head(3);
            mat(3, 0) = 0;
            mat(3, 1) = 0;
            mat(3, 2) = 0;
            mat(3, 3) = 1;

            mat.block<3, 3>(0, 0) = (Eigen::AngleAxisd(euler(5), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler(4), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler(3), Eigen::Vector3d::UnitX()))
                                        .matrix();
        }

        Vector6d GetTransInBase(const Vector6d &p1, const Vector6d &p0)
        {
            // Computes p0->p1 transformation IN BASE FRAME
            // idea: 1. use reference pose roation matrix and actual pose rotation matrix to calculate the rotation error
            //       2. then transform the rotation error matrix to axis angle form for controller
            Vector6d pose_error;
            pose_error(0) = p1(0) - p0(0);
            pose_error(1) = p1(1) - p0(1);
            pose_error(2) = p1(2) - p0(2);

            Eigen::Vector3f ref_rot(p0(3), p0(4), p0(5));
            Eigen::Vector3f actual_rot(p1(3), p1(4), p1(5));

            Eigen::AngleAxisf ref_angax(ref_rot.norm(), ref_rot.normalized());
            Eigen::AngleAxisf actual_angax(actual_rot.norm(), actual_rot.normalized());

            Eigen::Matrix3f ref_mat, actual_mat, error_mat;
            ref_mat = ref_angax;
            actual_mat = actual_angax;

            // p0->p1(base) * base->p0 = base->p1
            error_mat = actual_mat * ref_mat.transpose();

            Eigen::AngleAxisf error_angax(error_mat);
            pose_error(3) = error_angax.axis()(0) * error_angax.angle();
            pose_error(4) = error_angax.axis()(1) * error_angax.angle();
            pose_error(5) = error_angax.axis()(2) * error_angax.angle();

            return pose_error;
        }

        void BatchApplyTrans(Eigen::MatrixXd &p_arr, const Vector6d &target)
        {
            try
            {
                // apply p_arr to target in base frame
                if (p_arr.rows() != 6)
                {
                    std::ostringstream ss;
                    ss << "BatchApplyTrans: Expected p_arr with 6 rows, got " << p_arr.rows();
                    throw std::runtime_error(ss.str());
                }

                p_arr.block(0, 0, 3, p_arr.cols()) += target.head(3).replicate(1, p_arr.cols());

                Eigen::Matrix3d p_rot_mat;
                Eigen::Matrix3d target_rot_mat{Eigen::AngleAxisd{target.tail(3).norm(), target.tail(3).normalized()}};
                Eigen::AngleAxisd result;
                for (int t = 0; t < p_arr.cols(); ++t)
                {
                    p_rot_mat = Eigen::AngleAxisd{(p_arr.block<3, 1>(3, t)).norm(), (p_arr.block<3, 1>(3, t)).normalized()};
                    result = p_rot_mat * target_rot_mat; // apply p_rot in base frame
                    p_arr.block<3, 1>(3, t) = result.axis() * result.angle();
                }
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Vector6d LeftDiff(const Vector6d &p0, const Vector6d &p1, const bool &reverse)
        {
            // diff * p0 = p1

            Eigen::Matrix4d m0, m1;
            PoseAngRadToTransMat(p0, m0);
            PoseAngRadToTransMat(p1, m1);
            Vector6d diff;

            // translation
            diff.head(3) = m1.block<3, 1>(0, 3) - m0.block<3, 1>(0, 3);

            // rotation
            Eigen::Matrix3d m0_rot = m0.block<3, 3>(0, 0);
            Eigen::Matrix3d m1_rot = m1.block<3, 3>(0, 0);
            Eigen::Matrix3d diff_rot = m1_rot * m0_rot.transpose();
            Eigen::AngleAxisd diff_angax(diff_rot);
            if (reverse)
            {
                if (diff_angax.angle() > 0)
                {
                    diff_angax.angle() -= 2 * M_PI;
                }
                else
                {
                    diff_angax.angle() += 2 * M_PI;
                }
            }
            diff.tail(3) = diff_angax.axis() * diff_angax.angle();

            std::cout << ">> diff_rot angle:\n"
                      << diff_angax.angle() << "\n";

            return diff;
        }

        Vector6d LeftApply(const Vector6d &p0, const Vector6d &diff)
        {
            Eigen::Matrix4d m0;
            PoseAngRadToTransMat(p0, m0);

            Eigen::Matrix4d m1 = m0;
            m1.block<3, 1>(0, 3) = m0.block<3, 1>(0, 3) + diff.head(3);
            Eigen::Matrix3d diff_rot(Eigen::AngleAxisd(diff.tail(3).norm(), diff.tail(3).normalized()));
            m1.block<3, 3>(0, 0) = diff_rot * m0.block<3, 3>(0, 0);

            Vector6d p1;
            TransMatToPoseAngRad(m1, p1);

            return p1;
        }

        /* -------------------------------------------------------------------------- */
        /*                                 Kinematics                                 */
        /* -------------------------------------------------------------------------- */

        Eigen::Matrix4d FKine(const VectorJd &q, const Eigen::MatrixXd &DH)
        {
            // Determine robot model from [1, 3] value of DH parameters
            std::string robot_model;
            if (std::abs(DH(1, 2) - 0.0) < 1e-6)
            {
                robot_model = "ur5";
            }
            else if (std::abs(DH(1, 2) - 0.040) < 1e-6)
            {
                robot_model = "gp7";
            }
            else if (std::abs(DH(1, 2) - 0.155) < 1e-6)
            {
                robot_model = "gp12";
            }

            return FKine(q, robot_model);
        }

        // Eigen::Matrix4d FKine(const VectorJd &q, const Eigen::MatrixXd &DH)
        // {
        //     // DH row: [ theta, d, a, alpha ]
        //     // row 0:                   base to joint 1
        //     // row [1, n_joint]:        joint i to i+1
        //     // row [n_joint+1, end]:    additional trans for until ee

        //     std::vector<std::function<Eigen::Matrix4d(double q)>> link_transform;
        //     int n_joint{q.rows()};

        //     link_transform.clear();
        //     // generate transform matrix including ee links
        //     for (uint i = 0; i < DH.rows(); ++i)
        //     {
        //         link_transform.emplace_back(
        //             [DH, i](double q)
        //             {
        //                 Eigen::Matrix4d trans_mat;
        //                 trans_mat << cos(q + DH(i, 0)), -sin(q + DH(i, 0)) * cos(DH(i, 3)), sin(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * cos(q + DH(i, 0)),
        //                     sin(q + DH(i, 0)), cos(q + DH(i, 0)) * cos(DH(i, 3)), -cos(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * sin(q + DH(i, 0)),
        //                     0, sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1),
        //                     0, 0, 0, 1;
        //                 return trans_mat;
        //             });
        //     }

        //     // base to joint 1
        //     auto f_transform = link_transform.at(0);
        //     auto trans_mat = f_transform(0.0f);

        //     // joint 1 to link n_joint
        //     for (uint i = 0; i < n_joint; ++i)
        //     {
        //         // i -> i+1, result is pose of joint i+1
        //         f_transform = link_transform.at(i + 1);
        //         trans_mat = trans_mat * f_transform(q(i));
        //     }

        //     // link n_joint to ee
        //     for (uint i = n_joint + 1; i < link_transform.size(); ++i)
        //     {
        //         f_transform = link_transform.at(i);
        //         trans_mat = trans_mat * f_transform(0.0f);
        //     }

        //     return trans_mat;
        // }

        Eigen::Matrix4d FKine(const VectorJd &q,
                              const std::vector<std::function<Eigen::Matrix4d(double q)>> &link_transform)
        {
            int n_joint{q.rows()};

            // base link
            auto f_transform = link_transform.at(0);
            auto trans_mat = f_transform(0.0f);

            // joint links
            for (uint i = 0; i < n_joint; ++i)
            {
                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform.at(i + 1);
                trans_mat = trans_mat * f_transform(q(i));
            }

            // ee links
            for (uint i = n_joint + 1; i < link_transform.size(); ++i)
            {
                f_transform = link_transform.at(i);
                trans_mat = trans_mat * f_transform(0.0f);
            }

            return trans_mat;
        }

        Eigen::MatrixXd Jacobi(const VectorJd &q, const Vector6d &pose, const Eigen::MatrixXd &DH)
        {
            int n_joint{q.rows()};
            Eigen::MatrixXd Jacobian(6, n_joint);
            // 3xN z direction of each joint (rotation axis), joint location
            Eigen::Matrix<double, 3, Eigen::Dynamic> zj_(3, n_joint), rj_(3, n_joint);

            std::vector<std::function<Eigen::Matrix4d(double q)>> link_transform;

            link_transform.clear();
            // generate transform matrix
            for (uint i = 0; i < n_joint + 1; ++i)
            {
                link_transform.emplace_back(
                    [DH, i](double q)
                    {
                        Eigen::Matrix4d trans_mat;
                        trans_mat << cos(q + DH(i, 0)), -sin(q + DH(i, 0)) * cos(DH(i, 3)), sin(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * cos(q + DH(i, 0)),
                            sin(q + DH(i, 0)), cos(q + DH(i, 0)) * cos(DH(i, 3)), -cos(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * sin(q + DH(i, 0)),
                            0, sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1),
                            0, 0, 0, 1;
                        return trans_mat;
                    });
            }

            // Pose of joint 1
            auto f_transform = link_transform.at(0);
            auto trans_mat = f_transform(0.0f);

            for (uint i = 0; i < n_joint; ++i)
            {
                // Joint location
                rj_.col(i) = trans_mat.block<3, 1>(0, 3);
                // Joint direction
                zj_.col(i) = trans_mat.block<3, 1>(0, 2);

                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform.at(i + 1);
                trans_mat = trans_mat * f_transform(q(i));

                // end effector geometric jacobian
                Jacobian.block<3, 1>(0, i) = zj_.col(i).cross(Eigen::Vector3d{pose.head(3)} - rj_.col(i));
                Jacobian.block<3, 1>(3, i) = zj_.col(i);
            }

            Eigen::Vector3d phi = pose.tail(3);
            Eigen::Matrix3d phi_skew = skew(phi);

            Eigen::Matrix3d E_inv = Eigen::Matrix3d::Identity() - 0.5 * phi_skew + (phi_skew * phi_skew / (phi.norm() * phi.norm())) * (1 - phi.norm() * sin(phi.norm()) / (2 * (1 - cos(phi.norm()))));

            Eigen::Matrix<double, 6, 6> geo2analytical;
            geo2analytical << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), E_inv;

            // std::cout << ">> q:\n" << q << "\n";
            // std::cout << ">> pose:\n" << pose << "\n";
            // std::cout << ">> geo2analytical:\n" << geo2analytical << "\n";
            // std::cout << ">> Jacobian:\n" << Jacobian << "\n";
            // std::cout << ">> geo2analytical * Jacobian:\n" << geo2analytical * Jacobian << "\n";

            // while (true)
            // {

            // }

            return geo2analytical * Jacobian;
        }

        Eigen::MatrixXd Jacobi(const VectorJd &q, const Vector6d &pose,
                               const std::vector<std::function<Eigen::Matrix4d(double q)>> &link_transform)
        {
            int n_joint{q.rows()};
            Eigen::MatrixXd Jacobian(6, n_joint);
            // 3xN z direction of each joint (rotation axis), joint location
            Eigen::Matrix<double, 3, Eigen::Dynamic> zj_(3, n_joint), rj_(3, n_joint);

            // Pose of joint 1
            auto f_transform = link_transform.at(0);
            auto trans_mat = f_transform(0.0f);

            for (uint i = 0; i < n_joint; ++i)
            {
                // Joint location
                rj_.col(i) = trans_mat.block<3, 1>(0, 3);
                // Joint direction
                zj_.col(i) = trans_mat.block<3, 1>(0, 2);

                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform.at(i + 1);
                trans_mat = trans_mat * f_transform(q(i));

                // end effector jacobian
                Jacobian.block<3, 1>(0, i) = zj_.col(i).cross(Eigen::Vector3d{pose.head(3)} - rj_.col(i));
                Jacobian.block<3, 1>(3, i) = zj_.col(i);
            }

            return Jacobian;
        }

        Vector6d IK(
            const cfslib::math::Vector6d &c_ref, const cfslib::math::Vector6d &c0,
            const cfslib::math::VectorJd &theta0, const Eigen::MatrixXd &DH,
            const double &Cartesian_thres, const double &convergence_thres,
            const int &max_iter)
        {
            Eigen::Matrix4d mat_ref;
            PoseAngRadToTransMat(c_ref, mat_ref);

            // Determine robot model from [1, 3] value of DH parameters
            std::string robot_model;
            if (std::abs(DH(1, 2) - 0.0) < 1e-6)
            {
                robot_model = "ur5";
            }
            else if (std::abs(DH(1, 2) - 0.040) < 1e-6)
            {
                robot_model = "gp7";
            }
            else if (std::abs(DH(1, 2) - 0.155) < 1e-6)
            {
                robot_model = "gp12";
            }
            else
            {
                throw std::invalid_argument("Unknown DH parameters");
            }

            return IKine(
                theta0, mat_ref, robot_model, Cartesian_thres, convergence_thres, max_iter);
        }

        Eigen::MatrixXd BatchFKine(const Eigen::MatrixXd &qq, const Eigen::MatrixXd &DH)
        {
            // qq [n_joint, n_pt]
            try
            {
                int n_pt{qq.cols()};
                Eigen::MatrixXd carts(CARTESIAN_DIMS, n_pt);
                Vector6d cart;
                for (uint i = 0; i < n_pt; ++i)
                {
                    TransMatToPoseAngRad(FKine(qq.col(i), DH), cart);
                    carts.col(i) = cart;
                }
                return carts;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        Eigen::MatrixXd JointAxes(const VectorJd &q, const Eigen::MatrixXd &DH)
        {
            int n_joint{q.rows()};
            // 3xN z direction of each joint (rotation axis)
            Eigen::Matrix<double, 3, Eigen::Dynamic> zj_(3, n_joint);

            std::vector<std::function<Eigen::Matrix4d(double q)>> link_transform;

            link_transform.clear();
            // generate transform matrix
            for (uint i = 0; i < n_joint + 1; ++i)
            {
                link_transform.emplace_back(
                    [DH, i](double q)
                    {
                        Eigen::Matrix4d trans_mat;
                        trans_mat << cos(q + DH(i, 0)), -sin(q + DH(i, 0)) * cos(DH(i, 3)), sin(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * cos(q + DH(i, 0)),
                            sin(q + DH(i, 0)), cos(q + DH(i, 0)) * cos(DH(i, 3)), -cos(q + DH(i, 0)) * sin(DH(i, 3)), DH(i, 2) * sin(q + DH(i, 0)),
                            0, sin(DH(i, 3)), cos(DH(i, 3)), DH(i, 1),
                            0, 0, 0, 1;
                        return trans_mat;
                    });
            }

            // Pose of joint 1
            auto f_transform = link_transform.at(0);
            auto trans_mat = f_transform(0.0f);

            for (uint i = 0; i < n_joint; ++i)
            {
                // Joint direction
                zj_.col(i) = trans_mat.block<3, 1>(0, 2);

                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform.at(i + 1);
                trans_mat = trans_mat * f_transform(q(i));
            }

            return zj_;
        }

        Eigen::MatrixXd JointAxes(const VectorJd &q,
                                  const std::vector<std::function<Eigen::Matrix4d(double q)>> &link_transform)
        {
            int n_joint{q.rows()};
            // 3xN z direction of each joint (rotation axis)
            Eigen::Matrix<double, 3, Eigen::Dynamic> zj_(3, n_joint);

            // Pose of joint 1
            auto f_transform = link_transform.at(0);
            auto trans_mat = f_transform(0.0f);

            for (uint i = 0; i < n_joint; ++i)
            {
                // Joint direction
                zj_.col(i) = trans_mat.block<3, 1>(0, 2);

                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform.at(i + 1);
                trans_mat = trans_mat * f_transform(q(i));
            }

            return zj_;
        }

        /* -------------------------------------------------------------------------- */
        /*                              Collision Check                               */
        /* -------------------------------------------------------------------------- */
        double DistLineSeg(const lineseg &line1, const lineseg &line2)
        {
            Eigen::Vector3d s1, e1, s2, e2;
            Eigen::Vector3d d1, d2, d12;

            s1 = line1.p1;
            e1 = line1.p2;
            s2 = line2.p1;
            e2 = line2.p2;
            d1 = e1 - s1;
            d2 = e2 - s2;
            d12 = s2 - s1;

            double D1 = d1.dot(d1);
            double D2 = d2.dot(d2);
            double S1 = d1.dot(d12);
            double S2 = d2.dot(d12);
            double R = d1.dot(d2);
            double den = D1 * D2 - pow(R, 2);
            double u = 0, t = 0, uf = 0;

            // One of the segments is a point
            if (D1 == 0 || D2 == 0)
            {
                if (D1 != 0)
                {
                    t = Clip(S1 / D1, 0.0, 1.0);
                }
                else if (D2 != 0)
                {
                    u = Clip(-S2 / D2, 0.0, 1.0);
                }
            }
            // Segments are parallel
            else if (den == 0)
            {
                u = -S2 / D2;
                uf = Clip(u, 0.0, 1.0);
                if (uf != u)
                {
                    t = (uf * R + S1) / D1;
                    t = Clip(t, 0.0, 1.0);
                    u = uf;
                }
            }
            // General case
            else
            {
                t = Clip((S1 * D2 - S2 * R) / den, 0.0, 1.0);
                u = (t * R - S2) / D2;
                uf = Clip(u, 0.0, 1.0);
                if (uf != u)
                {
                    t = Clip((uf * R + S1) / D1, 0.0, 1.0);
                    u = uf;
                }
            }

            // Compute the distance, given t and u
            Eigen::MatrixXd dist = d1 * t - d2 * u - d12;
            return dist.norm();
        }

        double DistCap2Cap(const Capsule &cap1, const Capsule &cap2)
        {
            lineseg axis1, axis2;
            double r1, r2;

            axis1.p1 = cap1.p.col(0);
            axis1.p2 = cap1.p.col(1);
            axis2.p1 = cap2.p.col(0);
            axis2.p2 = cap2.p.col(1);
            r1 = cap1.r;
            r2 = cap2.r;

            double dist = DistLineSeg(axis1, axis2);
            return dist - r1 - r2;
        }

        double DistCap2BB(const Capsule &cap, const BoundingBox &bb)
        {
            lineseg cap_axis, bb_border;
            double cap_r, dist;
            double min_dist = INFINITY;

            cap_axis.p1 = cap.p.col(0);
            cap_axis.p2 = cap.p.col(1);
            cap_r = cap.r;

            Eigen::Vector3d n1, p11, p12, n2, p21, p22, n3, p31, p32, vert1, vert2;
            n1 = bb.n1;
            p11 = bb.p1_1;
            p12 = bb.p1_2;
            n2 = bb.n2;
            p21 = bb.p2_1;
            p22 = bb.p2_2;
            n3 = bb.n3;
            p31 = bb.p3_1;
            p32 = bb.p3_2;

            Eigen::MatrixXd bb_vertices(3, 8);
            bb_vertices.col(0) = p11 + n2.dot(p21 - p11) * n2 + n3.dot(p31 - p11) * n3;
            bb_vertices.col(1) = p11 + n2.dot(p22 - p11) * n2 + n3.dot(p31 - p11) * n3;
            bb_vertices.col(2) = p11 + n2.dot(p21 - p11) * n2 + n3.dot(p32 - p11) * n3;
            bb_vertices.col(3) = p11 + n2.dot(p22 - p11) * n2 + n3.dot(p32 - p11) * n3;
            bb_vertices.col(4) = p12 + n2.dot(p21 - p12) * n2 + n3.dot(p31 - p12) * n3;
            bb_vertices.col(5) = p12 + n2.dot(p22 - p12) * n2 + n3.dot(p31 - p12) * n3;
            bb_vertices.col(6) = p12 + n2.dot(p21 - p12) * n2 + n3.dot(p32 - p12) * n3;
            bb_vertices.col(7) = p12 + n2.dot(p22 - p12) * n2 + n3.dot(p32 - p12) * n3;

            for (int i = 0; i < bb_vertices.cols(); i++)
            {
                vert1 = bb_vertices.col(i);
                for (int j = i + 1; j < bb_vertices.cols(); j++)
                {
                    vert2 = bb_vertices.col(j);
                    if (VecPerp(n1, vert1 - vert2) || VecPerp(n2, vert1 - vert2) || VecPerp(n3, vert1 - vert2))
                    {
                        bb_border.p1 = vert1;
                        bb_border.p2 = vert2;
                        dist = DistLineSeg(bb_border, cap_axis) - cap_r;
                        if (dist < min_dist)
                        {
                            min_dist = dist;
                            if (min_dist <= 0)
                            {
                                return min_dist;
                            }
                        }
                    }
                }
            }
            return min_dist;
        }

        /* -------------------------------------------------------------------------- */
        /*                             Optimization Helper                            */
        /* -------------------------------------------------------------------------- */

        bool ConstraintsSatisfied(const Eigen::MatrixXd &x, const Eigen::MatrixXd A, const Eigen::MatrixXd &b,
                                  const Eigen::MatrixXd &Aeq, const Eigen::MatrixXd &beq)
        {

            std::cout << "\n";

            bool valid = true;

            if ((A * x - b).maxCoeff() > 1e-3)
            {

                int max_row, max_col;
                double max_coeff = (A * x - b).maxCoeff(&max_row, &max_col);

                std::cout << ">> Ineq constraints not satisfied\n";
                std::cout << ">> A shape: " << A.rows() << "x" << A.cols() << "\n";
                std::cout << ">> b shape: " << b.rows() << "x" << b.cols() << "\n";
                std::cout << ">> x shape: " << x.rows() << "x" << x.cols() << "\n";
                std::cout << ">> A * x - b max coeff: " << max_coeff << "\n";
                std::cout << ">> Max coeff index: (" << max_row << ", " << max_col << ")\n";

                valid = false;
            }

            if (!(Aeq * x).isApprox(beq))
            {
                double tolerance = 1e-8; // Define a small tolerance for floating-point comparisons
                Eigen::VectorXd diff = Aeq * x - beq;

                std::cout << ">> Eq constraints not satisfied\n";
                std::cout << ">> Aeq shape: " << Aeq.rows() << "x" << Aeq.cols() << "\n";
                std::cout << ">> beq shape: " << beq.rows() << "x" << beq.cols() << "\n";
                std::cout << ">> x shape: " << x.rows() << "x" << x.cols() << "\n";

                for (int i = 0; i < diff.size(); ++i)
                {
                    if (std::abs(diff(i)) > tolerance)
                    {
                        std::cout << ">> Constraint not satisfied at index " << i
                                  << ": Aeq * x = " << (Aeq * x)(i)
                                  << ", beq = " << beq(i)
                                  << ", difference = " << diff(i) << "\n";
                    }
                }

                valid = false;
            }

            return valid;
        }

        /* -------------------------------------------------------------------------- */
        /*                            Common Operations                               */
        /* -------------------------------------------------------------------------- */

        bool ApproxEq(const Eigen::VectorXd &x, const Eigen::VectorXd &y, const double &thres)
        {
            return (bool)((x - y).norm() < thres);
        }

        bool ApproxEqNum(const double &a, const double &b, const double &thres)
        {
            return (bool)(abs(a - b) < thres);
        }

        bool VecPerp(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2)
        {
            return (bool)ApproxEqNum(vec1.dot(vec2), 0, 0.0001);
        }

        std::string print_eigen_mat(const Eigen::MatrixXd &mat)
        {
            std::stringstream ss;
            for (int i = 0; i < mat.rows(); ++i)
            {
                ss << "[";
                for (int j = 0; j < mat.cols(); ++j)
                {
                    ss << mat(i, j);
                    // Add a comma after each element except the last in each row
                    if (j < mat.cols() - 1)
                    {
                        ss << ", ";
                    }
                }
                ss << "],\n"; // Move to the next line after each row
            }
            return ss.str();
        }

        Eigen::Matrix4d FKine(const Eigen::VectorXd &q, const std::string &robot_model)
        {
            if (robot_model == "gp7")
            {
                double eerot[9], eetrans[3];
                double joints[ikfastgp7::GetNumJoints()];
                for (unsigned int i = 0; i < ikfastgp7::GetNumJoints(); i++)
                {
                    joints[i] = q[i];
                }
                ikfastgp7::ComputeFk(joints, eetrans, eerot);
                Eigen::Matrix4d transform3d;
                transform3d << eerot[0], eerot[1], eerot[2], eetrans[0],
                    eerot[3], eerot[4], eerot[5], eetrans[1],
                    eerot[6], eerot[7], eerot[8], eetrans[2],
                    0.0, 0.0, 0.0, 1.0;
                Eigen::Matrix4d fixbase;
                fixbase << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, -0.33,
                    0, 0, 0, 1;
                Eigen::Matrix4d fixtool;
                fixtool << 0, 0, -1, -0.07,
                    0, -1, 0, 0,
                    -1, 0, 0, -0.4,
                    0, 0, 0, 1;
                return fixbase * transform3d * fixtool;
            }
            else if (robot_model == "gp12")
            {
                double eerot[9], eetrans[3];
                double joints[ikfastgp12::GetNumJoints()];
                for (unsigned int i = 0; i < ikfastgp12::GetNumJoints(); i++)
                {
                    joints[i] = q[i];
                }
                ikfastgp12::ComputeFk(joints, eetrans, eerot);
                Eigen::Matrix4d transform3d;
                transform3d << eerot[0], eerot[1], eerot[2], eetrans[0],
                    eerot[3], eerot[4], eerot[5], eetrans[1],
                    eerot[6], eerot[7], eerot[8], eetrans[2],
                    0.0, 0.0, 0.0, 1.0;
                Eigen::Matrix4d fixbase;
                fixbase << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0.05,
                    0, 0, 0, 1;
                Eigen::Matrix4d fixtool;
                fixtool << 0, 0, 1, 0,
                    0, -1, 0, 0,
                    1, 0, 0, 0,
                    0, 0, 0, 1;
                return fixbase * transform3d * fixtool;
            }
        }

        cfslib::math::Vector6d _IKnumeric(
            const cfslib::math::Vector6d &c_ref, const cfslib::math::Vector6d &c0,
            const cfslib::math::VectorJd &theta0, const Eigen::MatrixXd &DH,
            const double &thres, const int &max_iter)
        {
            double kp = 0.2;
            double kd = 0.01;
            int cnt = 0;
            cfslib::math::Vector6d c_var, c_err, c_err_prev, c_delta;
            cfslib::math::VectorJd theta_var{theta0};
            c_var = c0;
            c_err = c_ref - c_var;
            c_err_prev = c_err;

            double max_delta_theta = -1;

            while (c_err.norm() > thres)
            {
                // std::cout << ">> _IK iter: " << cnt << "\n";
                // check Jacobian singularity
                Eigen::MatrixXd J = cfslib::math::Jacobi(theta_var, c_var, DH);

                // Perform SVD on the Jacobian
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::VectorXd singularValues = svd.singularValues();
                Eigen::MatrixXd U = svd.matrixU();
                Eigen::MatrixXd V = svd.matrixV();

                // perturb joint until out of singularity
                int cnt_singular = 0;
                while (singularValues.minCoeff() < 0.01)
                {
                    theta_var = theta_var + V.col(5) * 0.1;
                    cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_var, DH), c_var);

                    J = cfslib::math::Jacobi(theta_var, c_var, DH);
                    svd.compute(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
                    singularValues = svd.singularValues();
                    U = svd.matrixU();
                    V = svd.matrixV();

                    // std::cout << "\n >>>>>>>> perturb \n";
                    // std::cout << ">> theta_var: " << theta_var.transpose() << "\n";
                    // std::cout << ">> c_var: " << c_var.transpose() << "\n";
                    // std::cout << ">> singularValues: " << singularValues.transpose() << "\n";

                    if (++cnt_singular > 100)
                    {
                        break;
                    }
                }

                c_err = c_ref - c_var;
                c_delta = kp * c_err + kd * (c_err - c_err_prev);

                // apply jacob and update c
                cfslib::math::VectorJd theta_delta = cfslib::math::PInv(J) * c_delta;
                theta_var = theta_var + theta_delta;
                cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_var, DH), c_var);

                c_err_prev = c_err;
                c_err = c_ref - c_var;

                /* ---------------------------------- debug --------------------------------- */

                // std::cout << "\n\n\n>> +++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
                // std::cout << ">> cnt: " << cnt << "\n";
                // std::cout << ">> c0:            " << math::print_eigen_mat(c0.transpose()) << "\n";
                // std::cout << ">> c_var:         " << math::print_eigen_mat(c_var.transpose()) << "\n";
                // std::cout << ">> c_ref:         " << math::print_eigen_mat(c_ref.transpose()) << "\n";
                // std::cout << ">> c_delta:       " << math::print_eigen_mat(c_delta.transpose()) << "\n";
                // std::cout << ">> theta0:        " << math::print_eigen_mat(theta0.transpose()) << "\n";
                // std::cout << ">> theta_delta:   " << math::print_eigen_mat(theta_delta.transpose()) << "\n";
                // std::cout << ">> theta_var:     " << math::print_eigen_mat(theta_var.transpose()) << "\n";
                // std::cout << ">> c_err:         " << math::print_eigen_mat(c_err.transpose()) << "\n";
                // std::cout << ">> c_err norm:    " << c_err.norm() << "\n";
                // std::cout << ">> Jacobi: \n" << J << "\n";
                // std::cout << ">> Jacobi SVD: \n" << math::print_eigen_mat(singularValues.transpose()) << "\n";
                // std::cout << ">> Jacobi U: \n" << U << "\n";
                // std::cout << ">> Jacobi_inv: \n" << cfslib::math::PInv(J) << "\n";

                // get out of singularity

                /* ---------------------------------- debug --------------------------------- */

                // if (theta_delta.norm() > max_delta_theta)
                // {
                //     max_delta_theta = theta_delta.norm();
                // }

                // if (max_delta_theta > 10)
                // {
                //     std::ostringstream ss;
                //     ss << ERR_HEADER << "IK: Exceeded max delta theta " << 10;
                //     throw std::runtime_error(ss.str());
                // }

                if (++cnt > max_iter)
                {
                    break;
                }
            }

            // std::cout << ">> converged in: " << cnt << " iterations\n";
            // std::cout << ">> max_delta_theta: " << max_delta_theta << "\n";

            return theta_var;
        }

        cfslib::math::Vector6d IKnumeric(
            const cfslib::math::Vector6d &c_ref, const cfslib::math::Vector6d &c0,
            const cfslib::math::VectorJd &theta0, const Eigen::MatrixXd &DH,
            const double &Cartesian_thres, const double &convergence_thres,
            const int &max_iter)
        {
            cfslib::math::Vector6d theta_guess{theta0}, c_guess{c0}, theta_sol{theta0}, c_sol{c0};
            cfslib::math::Vector6d c_err;

            cfslib::math::Vector6d theta_optimal;
            cfslib::math::Vector6d theta_optimal_prev;
            for (int i = 0; i < 6; ++i)
            {
                theta_optimal(i) = 1e6;
                theta_optimal_prev(i) = 1e6;
            }
            bool found_feasible = false;

            int guess_max = 100;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<double> dist(-M_PI / 2.0, M_PI / 2.0);

            for (int guess_i = 0; guess_i < guess_max; ++guess_i)
            {
                // std::cout << ">> --------------------------++++ \n";
                // std::cout << ">> Guess " << guess_i << "\n";

                // randomize theta_guess
                for (int i = 0; i < theta_guess.rows(); ++i)
                {
                    theta_guess(i) = theta0(i) + dist(gen);
                }
                cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_guess, DH), c_guess);

                // solve IK
                theta_sol = cfslib::math::_IKnumeric(c_ref, c_guess, theta_guess, DH, Cartesian_thres, max_iter);

                // check solution
                cfslib::math::TransMatToPoseAngRad(cfslib::math::FKine(theta_sol, DH), c_sol);
                c_err = c_ref - c_sol;

                if (c_err.norm() < Cartesian_thres && theta_sol(2) < M_PI_2 && (theta_sol - theta0).cwiseAbs().maxCoeff() < M_PI * 3.0 / 4.0)
                {
                    // Cartesian accurate
                    // std::cout << "-------------------------------------------\n";
                    // std::cout << ">> Found feasible solution at guess " << guess_i << ".\n";
                    // std::cout << ">> c_ref: " << math::print_eigen_mat(c_ref.transpose()) << "\n";
                    // std::cout << ">> c_sol: " << math::print_eigen_mat(c_sol.transpose()) << "\n";
                    // std::cout << ">> c_err: " << math::print_eigen_mat(c_err.transpose()) << "\n";

                    if ((theta_sol - theta0).norm() < (theta_optimal - theta0).norm())
                    {

                        // better solution
                        found_feasible = true;

                        // backup
                        theta_optimal_prev = theta_optimal;

                        // update
                        theta_optimal = theta_sol;

                        // std::cout << ">> Found better solution at guess " << guess_i << "\n";
                        // std::cout << ">> theta_optimal:         " << cfslib::math::print_eigen_mat(theta_optimal.transpose()) << "\n";
                        // std::cout << ">> theta_optimal_prev:    " << cfslib::math::print_eigen_mat(theta_optimal_prev.transpose()) << "\n";
                        // std::cout << ">> optimal diff:          " << (theta_optimal - theta_optimal_prev).norm() << "\n";
                        // std::cout << ">> diff with initial:     " << (theta_optimal - theta0).norm() << "\n";
                        // std::cout << ">> max joint diff:        " << (theta_optimal - theta0).cwiseAbs().maxCoeff() << "\n";

                        // check termination
                        if ((theta_optimal - theta_optimal_prev).norm() < convergence_thres)
                        {
                            // std::cout << ">> Converged\n";
                            break;
                        }
                    }
                }
            }

            std::cout << ">> IK error: " << c_err.norm() << ", convergence: " << (theta_optimal - theta_optimal_prev).norm() << "\n";

            return theta_optimal;
        }

        cfslib::math::Vector6d IKine(
            const cfslib::math::VectorJd &theta0, const Eigen::Matrix4d &mat_ref,
            const std::string &robot_model,
            const double &Cartesian_thres, const double &convergence_thres,
            const int &max_initializations)
        {
            if (robot_model == "gp7")
            {
                // fix base and tool
                Eigen::Matrix4d fixbase;
                fixbase << 1, 0, 0, 0.0,
                    0, 1, 0, 0,
                    0, 0, 1, 0.33,
                    0, 0, 0, 1;
                Eigen::Matrix4d fixtool;
                fixtool << 0, 0, -1, -0.4,
                    0, -1, 0, 0,
                    -1, 0, 0, -0.07,
                    0, 0, 0, 1;
                Eigen::Matrix4d T = fixbase * mat_ref * fixtool;

                double eerot[9], eetrans[3];
                double joints[ikfastgp7::GetNumJoints()];

                // Copy current joint values
                for (unsigned int i = 0; i < ikfastgp7::GetNumJoints(); i++)
                {
                    joints[i] = theta0[i];
                }

                // Extract rotation and translation from transform matrix
                eerot[0] = T(0, 0);
                eerot[1] = T(0, 1);
                eerot[2] = T(0, 2);
                eerot[3] = T(1, 0);
                eerot[4] = T(1, 1);
                eerot[5] = T(1, 2);
                eerot[6] = T(2, 0);
                eerot[7] = T(2, 1);
                eerot[8] = T(2, 2);
                eetrans[0] = T(0, 3);
                eetrans[1] = T(1, 3);
                eetrans[2] = T(2, 3);

                IkSolutionList<double> solutions;
                std::vector<double> vfree(ikfastgp7::GetNumFreeParameters());

                bool success = ikfastgp7::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : nullptr, solutions);
                // success = false;
                if (success)
                {
                    unsigned int num_solutions = solutions.GetNumSolutions();
                    double best_dist = -1;
                    int best_solution_idx = -1;
                    std::vector<double> sol_values(ikfastgp7::GetNumJoints());

                    // Find closest solution to current joint values
                    for (unsigned int i = 0; i < num_solutions; ++i)
                    {
                        const IkSolutionBase<double> &sol = solutions.GetSolution(i);
                        std::vector<double> vsolfree(sol.GetFree().size());
                        sol.GetSolution(&sol_values[0], vsolfree.size() > 0 ? &vsolfree[0] : nullptr);

                        double dist = 0.0;
                        for (unsigned int j = 0; j < sol_values.size(); ++j)
                        {
                            double diff = sol_values[j] - theta0[j];
                            double min_diff = std::min({diff * diff,
                                                        (diff + 2 * M_PI) * (diff + 2 * M_PI),
                                                        (diff - 2 * M_PI) * (diff - 2 * M_PI)});
                            dist += min_diff;
                        }

                        if (best_dist < 0 || dist < best_dist)
                        {
                            best_dist = dist;
                            best_solution_idx = i;
                        }
                    }

                    if (best_solution_idx >= 0)
                    {
                        const IkSolutionBase<double> &sol = solutions.GetSolution(best_solution_idx);
                        std::vector<double> vsolfree(sol.GetFree().size());
                        sol.GetSolution(&sol_values[0], vsolfree.size() > 0 ? &vsolfree[0] : nullptr);

                        cfslib::math::Vector6d result;
                        for (unsigned int j = 0; j < sol_values.size(); ++j)
                        {
                            result[j] = sol_values[j];
                        }
                        return result;
                    }
                }
                else
                {
                    // use numerical
                    Eigen::MatrixXd DH(8, 4);
                    DH << 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.040, -1.5707963267948966,
                        1.5707963267948966, 0.0, -0.445, 3.14159265359,
                        0.0, 0.0, -0.040, 1.5707963267948966,
                        0.0, -0.44, 0.0, -1.5707963267948966,
                        0.0, 0.0, 0.0, 1.5707963267948966,
                        0.0, -0.080, 0.0, 0.0,
                        0, 0, 0, 0;
                    cfslib::math::Vector6d c_ref, c0;
                    cfslib::math::TransMatToPoseAngRad(mat_ref, c_ref);
                    cfslib::math::TransMatToPoseAngRad(FKine(theta0, "gp7"), c0);
                    return IKnumeric(
                        c_ref, c0, theta0, DH, Cartesian_thres, convergence_thres, max_initializations);
                }
                throw std::runtime_error("IK failed to find solution for GP7");
            }
            else if (robot_model == "gp12")
            {
                // fix base and tool
                Eigen::Matrix4d fixbase;
                fixbase << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, -0.05,
                    0, 0, 0, 1;
                Eigen::Matrix4d fixtool;
                fixtool << 0, 0, 1, 0,
                    0, -1, 0, 0,
                    1, 0, 0, 0,
                    0, 0, 0, 1;
                Eigen::Matrix4d T = fixbase * mat_ref * fixtool;

                double eerot[9], eetrans[3];
                double joints[ikfastgp12::GetNumJoints()];
                for (unsigned int i = 0; i < ikfastgp12::GetNumJoints(); i++)
                {
                    joints[i] = theta0[i];
                }

                eerot[0] = T(0, 0);
                eerot[1] = T(0, 1);
                eerot[2] = T(0, 2);
                eerot[3] = T(1, 0);
                eerot[4] = T(1, 1);
                eerot[5] = T(1, 2);
                eerot[6] = T(2, 0);
                eerot[7] = T(2, 1);
                eerot[8] = T(2, 2);
                eetrans[0] = T(0, 3);
                eetrans[1] = T(1, 3);
                eetrans[2] = T(2, 3);

                IkSolutionList<double> solutions;
                std::vector<double> vfree(ikfastgp12::GetNumFreeParameters());

                bool success = ikfastgp12::ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : nullptr, solutions);
                // success = false;

                if (success)
                {
                    // Same solution selection logic as GP7
                    unsigned int num_solutions = solutions.GetNumSolutions();
                    double best_dist = -1;
                    int best_solution_idx = -1;
                    std::vector<double> sol_values(ikfastgp12::GetNumJoints());

                    for (unsigned int i = 0; i < num_solutions; ++i)
                    {
                        const IkSolutionBase<double> &sol = solutions.GetSolution(i);
                        std::vector<double> vsolfree(sol.GetFree().size());
                        sol.GetSolution(&sol_values[0], vsolfree.size() > 0 ? &vsolfree[0] : nullptr);

                        double dist = 0.0;
                        for (unsigned int j = 0; j < sol_values.size(); ++j)
                        {
                            double diff = sol_values[j] - theta0[j];
                            double min_diff = std::min({diff * diff,
                                                        (diff + 2 * M_PI) * (diff + 2 * M_PI),
                                                        (diff - 2 * M_PI) * (diff - 2 * M_PI)});
                            dist += min_diff;
                        }

                        if (best_dist < 0 || dist < best_dist)
                        {
                            best_dist = dist;
                            best_solution_idx = i;
                        }
                    }

                    if (best_solution_idx >= 0)
                    {
                        const IkSolutionBase<double> &sol = solutions.GetSolution(best_solution_idx);
                        std::vector<double> vsolfree(sol.GetFree().size());
                        sol.GetSolution(&sol_values[0], vsolfree.size() > 0 ? &vsolfree[0] : nullptr);

                        cfslib::math::Vector6d result;
                        for (unsigned int j = 0; j < sol_values.size(); ++j)
                        {
                            result[j] = sol_values[j];
                        }
                        return result;
                    }
                }
                else
                {
                    // use numerical
                    Eigen::MatrixXd DH(8, 4);
                    DH << 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.155, -1.5707963267948966,
                        1.5707963267948966, 0.0, -0.614, 3.14159265359,
                        0.0, 0.0, -0.200, 1.5707963267948966,
                        0.0, -0.64, 0.0, -1.5707963267948966,
                        0.0, 0.0, 0.0, 1.5707963267948966,
                        0.0, -0.100, 0.0, 0.0,
                        0, 0, 0, 0;
                    cfslib::math::Vector6d c_ref, c0;
                    cfslib::math::TransMatToPoseAngRad(mat_ref, c_ref);
                    cfslib::math::TransMatToPoseAngRad(FKine(theta0, "gp7"), c0);
                    return IKnumeric(
                        c_ref, c0, theta0, DH, Cartesian_thres, convergence_thres, max_initializations);
                }
                throw std::runtime_error("IK failed to find solution for GP12");
            }

            throw std::invalid_argument("Invalid robot model");
        }
    }
}