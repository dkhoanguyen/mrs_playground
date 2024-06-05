classdef Flocking
    properties
        
        %% Constant
        C1_ALPHA=0.3;
        C2_ALPHA=0.2*sqrt(C1_ALPHA);
        C1_BETA=2;
        C2_BETA=0.2*sqrt(C1_BETA);
        C1_GAMMA=0.5;
        C2_GAMMA=0.02*sqrt(C1_GAMMA);
        
        %% Parameters
        dt;
        sensingRange;
        dangerRange;
        maxV;
        distance;
    end
    
    methods (Access=public)
        function obj=Flocking(dt, sensingRange, dangerRange, maxV, distance)
            obj.dt = dt;
            obj.sensingRange = sensingRange;
            obj.dangerRange = dangerRange;
            obj.maxV = maxV;
            obj.distance = distance;
        end
        
        function newStates = step(obj,animalStates, robotStates)
            uLocalClustering = obj.localClustering(animalStates,robotStates,k=1);
            uflocking = obj.flocking(animalStates,robotStates);
            % densities = obj.herdDensity(animalStates,robotStates);
            
            qDot = uflocking + uLocalClustering;
            
            avoidanceV = zeros(size(animalStates, 1), 2);
            
            % % Calculate avoidance velocities
            % for idx = 1:size(animalStates, 1)
            %     qi = animalStates(idx, 1:2);
            %     u_pred = obj.predatorAvoidanceTerm(qi, obj.dangerRange, 4,robotStates);
            %     avoidanceV(idx, :) = avoidanceV(idx, :) + u_pred + densities(idx, 1:2);
            % end
            % 
            % Update herd states with qdot
            animalStates(:, 3:4) = qDot * obj.dt + avoidanceV;
            
            
            % Update positions using the velocities
            pdot = animalStates(:, 3:4);
            animalStates(:, 1:2) = animalStates(:, 1:2) + pdot * obj.dt;
            newStates = animalStates;
        end
    end
    
    methods (Access=private)
        function allAlpha = flocking(obj, animalStates, robotStates)
            allAlpha = zeros(size(animalStates, 1), 2);
            alphaAdjMatrix = obj.getAlphaAdjacencyMatrix(animalStates,obj.sensingRange);
            
            for idx=1:size(animalStates, 1)
                % Flocking terms
                neighborIdxs = find(alphaAdjMatrix(idx, :));
                uAlpha = obj.calcFlockingControl(idx, neighborIdxs, animalStates);
                
                allAlpha(idx, :) = uAlpha;
            end
        end
        
        function allGamma = localClustering(obj, animalStates, robotStates, k)
            % Calculate the adjacency matrix
            adjMatrix = obj.getAlphaAdjacencyMatrix(animalStates,obj.distance);
            
            % Create a graph
            adjGraph = graph(adjMatrix);
            
            % Find clusters
            clusterIdxs = conncomp(adjGraph, 'OutputForm', 'cell');
            
            clusters = {};
            clusterIdxList = {};
            
            for i = 1:numel(clusterIdxs)
                cluster = [];
                cluster_indx = [];
                
                for j = 1:numel(clusterIdxs{i})
                    cluster = [cluster; animalStates(clusterIdxs{i}(j), :)];
                end
                
                if numel(clusterIdxs{i}) == 1
                    continue;
                end
                
                cluster_nodes = [];
                for j = 1:numel(clusterIdxs{i})
                    cluster_nodes = [cluster_nodes; animalStates(clusterIdxs{i}(j), :)];
                    cluster_indx = [cluster_indx; clusterIdxs{i}(j)];
                end
                
                clusters{end+1} = cluster_nodes;
                clusterIdxList{end+1} = cluster_indx;
            end
            
            % Perform local flocking with local cluster
            allGamma = zeros(size(animalStates, 1), 2);
            
            for i = 1:numel(clusters)
                if numel(clusters) == 1
                    continue;
                end
                
                localClusterStates = [];
                for j = 1:size(clusters{i}, 1)
                    localClusterStates = [localClusterStates; clusters{i}(j, :)];
                end
                
                for idx = 1:size(localClusterStates, 1)
                    qi = localClusterStates(idx, 1:2);
                    pi = localClusterStates(idx, 3:4);
                    
                    thisIndx = clusterIdxList{i}(idx);
                    
                    % Group consensus term
                    clusterMean = sum(localClusterStates(:, 1:2), 1) / size(localClusterStates, 1);
                    
                    target = clusterMean;
                    gain = k;
                    
                    % Apply selfish animal theory when in danger
                    totalPredatorInRange = 0;
                    robotMean = zeros(1, 2);
                    avoidVel = zeros(1, 2);
                    
                    for shepherdIndx = 1:size(robotStates, 1)
                        si = robotStates(shepherdIndx, 1:2);
                        if norm(qi - si) <= obj.dangerRange
                            gain = 1;
                            robotMean = robotMean + si;
                            totalPredatorInRange = totalPredatorInRange + 1;
                        end
                    end
                    
                    if totalPredatorInRange > 0
                        robotMean = robotMean / totalPredatorInRange;
                        avoidVel = 2 * (clusterMean - robotMean)/norm(clusterMean - robotMean);
                    end
                    
                    uGamma = gain * obj.groupObjectiveTerm(...
                        obj.C1_GAMMA, ...
                        obj.C2_GAMMA, ...
                        target, qi, avoidVel, pi ...
                        );
                    
                    allGamma(thisIndx, :) = uGamma;
                end
            end
        end
        
        function uPredAvoid = predatorAvoidanceTerm(si, r, k, robotStates)
            % Initialize the avoidance velocity
            siDot = zeros(1, 2);
            total = 0;
            
            % Loop through each robot to calculate avoidance term
            for idx = 1:length(obj.robots)
                robotPose = robotStates(1:2)';
                if norm(robotPose - si) <= r
                    siDot = siDot - k * (robotPose - si)/norm(robotPose - si);
                    total = total + 1;
                end
            end
            % Normalize if any robots were within range
            if total == 0
                return
            end
            uPredAvoid = siDot / total;
        end
        
        function herdDensities = herdDensity(obj, animalStates, robotStates)
            % Initialize the herd densities matrix
            herdDensities = zeros(size(animalStates, 1, 2));
            
            % Compute the alpha adjacency matrix
            alphaAdjMatrix = obj.getAlphaAdjacencyMatrix(animalStates, obj.distance * 1.5);
            
            % Loop through each animal to calculate density
            for idx = 1:size(animalStates, 1)
                % Get the indices of the neighbors
                neighborIdxs = find(alphaAdjMatrix(idx, :));
                
                % Calculate the density
                density = obj.calcDensity(idx, neighborIdxs, animalStates);
                
                % Normalize the density
                herdDensities(idx, :) = density/norm(density);
            end
        end
    end
    
    methods (Access=private)
        function density = calcDensity(obj, idx, neighborIdxs, animalStates)
            % Extract the position of the current animal
            qi = animalStates(idx, 1:2);
            
            % Initialize the density vector
            density = zeros(1, 2);
            
            % Check if there are any neighbors
            if ~isempty(neighborIdxs)
                % Extract the positions of the neighbors
                qj = animalStates(neighborIdxs, 1:2);
                
                % Calculate the density
                density = obj.density(qi, qj, 0.375);
            end
        end
        function wSum = density(obj, si, sj, k)
            % Initialize the weighted sum vector
            wSum = zeros(1, 2);
            
            % Loop through each neighbor's position
            for i = 1:size(sj, 1)
                sij = sj(i, :) - si;
                w = (1 / (1 + k * norm(sij))) * sij/norm(sij);
                wSum = wSum + w;
            end
        end
        
        
        function result = groupObjectiveTerm(obj, c1, c2, pos, qi, vel, pi)
            % Group objective term
            result = -c1 * MathUtils.sigma_1(qi - pos) - c2 * (pi - vel);
        end
        
        function uAlpha = calcFlockingControl(obj, idx, neighborsIdxs, animalStates)
            % Extract the position and velocity of the current animal
            qi = animalStates(idx, 1:2);
            pi = animalStates(idx, 3:4);
            
            % Initialize the flocking control term
            uAlpha = zeros(1, 2);
            
            % Check if there are any neighbors
            if sum(neighborsIdxs) > 0
                % Extract the positions and velocities of the neighbors
                qj = animalStates(neighborsIdxs, 1:2);
                pj = animalStates(neighborsIdxs, 3:4);
                
                % Compute the gradient term
                alphaGrad = obj.gradientTerm(...
                    obj.C2_ALPHA, qi, qj, obj.distance, obj.distance);
                
                % Compute the velocity consensus term
                alphaConsensus = obj.velocityConsensusTerm(...
                    obj.C2_ALPHA, qi, qj, pi, pj, obj.distance);
                
                % Sum the gradient and consensus terms
                uAlpha = alphaGrad + alphaConsensus;
            end
        end
    end
    
    methods (Access=private)
        function adjMatrix = getAlphaAdjacencyMatrix(obj, animalStates, r)
            % Initialize the adjacency matrix
            n = size(animalStates, 1);
            adjMatrix = false(n, n);
            
            % Compute the adjacency matrix based on the distance threshold r
            for i = 1:n
                distances = sqrt(sum((animalStates(i, 1:2) - animalStates(:, 1:2)).^2, 2));
                adjMatrix(i, :) = distances <= r;
            end
            
            % Remove self-connections by setting the diagonal to false
            adjMatrix(logical(eye(size(adjMatrix)))) = false;
        end
        
        function alphaGrad = gradientTerm(obj, c, qi, qj, r, d)
            % Compute the gradient term
            n_ij = obj.getNIJ(qi, qj);
            phiAlpha = MathUtils.phi_alpha(MathUtils.sigma_norm(qj - qi), r, d);
            alphaGrad = c * sum(phiAlpha .* n_ij, 1);
        end
        
        function alphaConsensus = velocityConsensusTerm(obj, c, qi, qj, pi, pj, r)
            % Compute the velocity consensus term
            a_ij = obj.getAIJ(qi, qj, r);
            alphaConsensus = c * sum(a_ij .* (pj - pi), 1);
        end
        
        function a_ij = getAIJ(obj, q_i, q_js, range)
            % Compute the adjacency influence term a_ij
            r_alpha = MathUtils.sigma_norm(range);
            a_ij = MathUtils.bump_function(MathUtils.sigma_norm(q_js - q_i) / r_alpha);
        end
        
        function n_ij = getNIJ(obj, q_i, q_js)
            % Compute the normalized gradient term n_ij
            n_ij = MathUtils.sigma_norm_grad(q_js - q_i);
        end
    end
end