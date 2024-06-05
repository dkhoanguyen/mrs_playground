classdef TestMathUtils < matlab.unittest.TestCase
    properties
        ExpectedResults
    end
    
    methods (TestClassSetup)
        function loadExpectedResults(testCase)
            testCase.ExpectedResults = load('expected_results.mat');
        end
    end
    
    methods (Test)
        function testSigma1(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.sigma_1(z);
            expected = testCase.ExpectedResults.sigma_1;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testSigmaNorm(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.sigma_norm(z);
            expected = testCase.ExpectedResults.sigma_norm;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testSigmaNormGrad(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.sigma_norm_grad(z);
            expected = testCase.ExpectedResults.sigma_norm_grad;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testBumpFunction(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.bump_function(z);
            expected = testCase.ExpectedResults.bump_function;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testPhi(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.phi(z);
            expected = testCase.ExpectedResults.phi;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testPhiAlpha(testCase)
            z = [0.5, 1.5, 2.5];
            result = MathUtils.phi_alpha(z);
            expected = testCase.ExpectedResults.phi_alpha;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
        
        function testNormalise(testCase)
            v = [3.0, 4.0];
            result = MathUtils.normalise(v);
            expected = testCase.ExpectedResults.normalise;
            testCase.verifyEqual(result, expected, 'AbsTol', 1e-10);
        end
    end
end
