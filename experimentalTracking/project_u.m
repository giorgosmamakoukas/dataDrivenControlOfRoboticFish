function u = project_u(u)
   rng default % For reproducibility
   opts = optimoptions(@fmincon,'Algorithm','sqp','Display','off');
   for i = 1: length(u(:,1))

   u1 = u(i,1); u2 = u(i,2);
   fun = @(a) (u1 - a(1)^2 * (3-3/2 * a(2)^2 - 3/8 *a(1)^2))^2 + (u2 - a(1)^2*a(2))^2;
   problem = createOptimProblem('fmincon','objective',...
       fun,'x0',[0;0],'lb',[0; -47*pi/180],'ub',[30*pi/180; 47*pi/180],'options',opts);  
%     gs = GlobalSearch;
%     gs = GlobalSearch('Display', 'off','NumStageOnePoints', 30, 'NumTrialPoints', 30, 'FunctionTolerance', 1e-3, 'MaxTime', 0.1);
   gs = GlobalSearch('Display', 'off');
   [x,f] = run(gs,problem);
   u(i,:) = x(:)';
   end
end