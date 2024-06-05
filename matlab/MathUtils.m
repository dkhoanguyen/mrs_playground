classdef MathUtils
    properties (Constant)
        EPSILON = 0.1
        H = 0.2
        A = 5
        B = 5
        C = abs(5 - 5) / sqrt(4 * 5 * 5) % phi
        R = 40
        D = 40
    end
    
    methods (Static)
        function result = sigma_1(z)
            result = z ./ sqrt(1 + z.^2);
        end
        
        function result = sigma_norm(z, e)
            if nargin < 2
                e = MathUtils.EPSILON;
            end
            norm_z = sqrt(sum(z.^2, 2));
            result = (sqrt(1 + e * norm_z.^2) - 1) / e;
        end
        
        function result = sigma_norm_grad(z, e)
            if nargin < 2
                e = MathUtils.EPSILON;
            end
            norm_z = sqrt(sum(z.^2, 2));
            result = z ./ sqrt(1 + e * norm_z.^2);
        end
        
        function result = bump_function(z, h)
            if nargin < 2
                h = MathUtils.H;
            end
            ph = zeros(size(z));
            idx1 = z <= 1;
            ph(idx1) = (1 + cos(pi * (z(idx1) - h) / (1 - h))) / 2;
            idx2 = z < h;
            ph(idx2) = 1;
            idx3 = z < 0;
            ph(idx3) = 0;
            result = ph;
        end
        
        function result = phi(z, a, b, c)
            if nargin < 2
                a = MathUtils.A;
            end
            if nargin < 3
                b = MathUtils.B;
            end
            if nargin < 4
                c = MathUtils.C;
            end
            result = ((a + b) * MathUtils.sigma_1(z + c) + (a - b)) / 2;
        end
        
        function result = phi_alpha(z, r, d)
            if nargin < 2
                r = MathUtils.R;
            end
            if nargin < 3
                d = MathUtils.D;
            end
            r_alpha = MathUtils.sigma_norm(r);
            d_alpha = MathUtils.sigma_norm(d);
            result = MathUtils.bump_function(z / r_alpha) .* MathUtils.phi(z - d_alpha);
        end
        
        function result = normalise(v, pre_computed)
            if nargin < 2
                n = norm(v);
            else
                n = pre_computed;
            end
            if n < 1e-13
                result = zeros(size(v));
            else
                result = v / n;
            end
        end
    end
end
