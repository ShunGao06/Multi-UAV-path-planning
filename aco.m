function bestTour = aco(data, maxIt, nAnt, q, tau0, alpha, beta, rho)

    % If some parameters are not provided, set default values
    if nargin < 8
        maxIt = 300;      % Maximum Number of Iterations
        nAnt = 40;        % Number of Ants (Population Size)
        q = 1;
        alpha = 1;        % Phromone Exponential Weight
        beta = 1;         % Heuristic Exponential Weight
        rho = 0.05;       % Evaporation Rate
        % Calculate tau0 based on the data provided
        distances = pdist2(data, data);
        tau0 = 10*q/(size(data, 1) * mean(distances(:))); % Initial Phromone
    end

    % Problem Definition
    nVar = size(data, 1); % Number of Decision Variables
    model.n = nVar;
    model.D = distances;

    % Define the CostFunction here
    CostFunction = @(tour) sum(sqrt(sum(diff(data(tour([1:end 1]), :)).^2, 2)));

    % ACO Parameters
    % (Already set above)

    % Initialization
    eta = 1./model.D;             % Heuristic Information Matrix
    tau = tau0 * ones(nVar, nVar); % Phromone Matrix
    % bestCost = inf;                % Initialize Best Cost

    % Ant Colony Matrix
    empty_ant.Tour = [];
    empty_ant.Cost = inf;

    % Ant Colony Matrix
    ant = repmat(empty_ant, nAnt, 1);

    % Best Ant
    BestSol = empty_ant;
    BestSol.Cost = inf;

    % ACO Main Loop
    for it = 1:maxIt
        % Move Ants
        for k = 1:nAnt
        
        ant(k).Tour = randi([1 nVar]);
        
        for l = 2:nVar
            
            i = ant(k).Tour(end);
            
            P = tau(i, :).^alpha.*eta(i, :).^beta;
            
            P(ant(k).Tour) = 0;
            
            P = P/sum(P);
            
            j = RouletteWheelSelection(P);
            
            ant(k).Tour = [ant(k).Tour j];
            
        end
        % Calculate the cost of the tour
        ant(k).Cost = CostFunction(ant(k).Tour);
        % Update BestSol if the current solution is better
        if ant(k).Cost<BestSol.Cost
            BestSol = ant(k);
        end
        
    end
    
    % Update Phromones
    for k = 1:nAnt
        
        tour = ant(k).Tour;
        
        tour = [tour tour(1)]; %#ok
        
        for l = 1:nVar

            i = tour(l);
            j = tour(l+1);
            
            tau(i, j) = tau(i, j)+q/ant(k).Cost;
            
        end
    end

    % Return Outputs
    bestTour = BestSol.Tour;
    % 
    % bestCost = bestSol.Cost;

end
