for K = 1
    %% PI controller gains
    tauC = 60/1000; % ms, controller time constant from Fig5 as above
    Kp = K * tauC;
    Ki = K;
    

    %% P(s) parameters
    I = 2119; % moment of inertia for magnificent hummingbird pitch
    tauP = 69.7/1000; % I/b, the open-loop time constant in ms

    %% Other Parameters
    simTime = 3 %seconds
    deltaT = 20/1000; % ms, sensory time delay, from Fig 5 in Cheng paper, chosen somewhat arbitrarily from allowable
    
    sim("BirdModel.slx");
    plot(ans.tout, ans.yout);
    title(['Response for K = ', num2str(K)]);
    xlabel('Time');
    ylabel('Output');
    saveas(gcf, ['Response_for_K_', num2str(K), '.png']);
end
    