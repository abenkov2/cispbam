#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <fstream>

template<class T, int dim>
class MassSpringSystem{
public:
    using TV = Eigen::Matrix<T,dim,1>;
    
    std::vector<Eigen::Matrix<int,2,1> > segments;
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    T youngs_modulus;
    T damping_coeff;
    std::vector<bool> node_is_fixed;
    std::vector<T> rest_length;

    MassSpringSystem()
    {}

    void evaluateSpringForces(std::vector<TV >& f)
    {
        // TODO: evaluate spring force
        f.clear();
        f.resize(x.size(), TV::Zero());
        for (unsigned long int i = 0; i < segments.size(); i++) {
            if (rest_length[i] != 0) {
                TV x_kj = x[segments[i][0]] - x[segments[i][1]];
                std::cout << "got here LALALALALALALALALALALA";
                TV x_jk = x[segments[i][1]] - x[segments[i][0]];
                TV d1 = x_kj.normalized();
                TV d2 = x_jk.normalized();
                f[segments[i][0]] += -youngs_modulus * (x_kj).norm() / rest_length[i] * d1;
                f[segments[i][1]] += -youngs_modulus * (x_jk).norm() / rest_length[i] * d2;

            }
        }

    }

    void evaluateDampingForces(std::vector<TV >& f)
    {
        // TODO: evaluate damping force
        f.clear();
        f.resize(x.size(), TV::Zero());
        

        for (unsigned long int i = 0; i < segments.size(); i++) {
            if (rest_length[i] != 0) {
            
                TV x_kj = x[segments[i][0]] - x[segments[i][1]];
                TV x_jk = x[segments[i][1]] - x[segments[i][0]];

                x_kj.normalize();
                x_jk.normalize();
                
                //TV d1 = x_kj.normalized();
                //TV d2 = x_jk.normalized();

                TV v_rel_kj = (v[segments[i][0]] - v[segments[i][1]]);
                TV v_rel_jk = (v[segments[i][1]] - v[segments[i][0]]);

                f[segments[i][0]] += -damping_coeff * (v_rel_kj.dot(x_kj)) * x_kj;
                f[segments[i][1]] += -damping_coeff * (v_rel_jk.dot(x_jk)) * x_jk;

            }
        }

    }

    void dumpPoly(std::string filename)
    {
        std::ofstream fs;
        fs.open(filename);
        fs << "POINTS\n";
        int count = 0;
        for (auto X : x) {
            fs << ++count << ":";
            for (int i = 0; i < dim; i++)
                fs << " " << X(i);
            if (dim == 2)
                fs << " 0";
            fs << "\n";
        }
        fs << "POLYS\n";
        count = 0;
        for (const Eigen::Matrix<int, 2, 1>& seg : segments)
            fs << ++count << ": " << seg(0) + 1 << " " << seg(1) + 1 << "\n"; // poly segment mesh is 1-indexed
        fs << "END\n";
        fs.close();
    }
};
