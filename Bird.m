for K = 1:20
    %% PI controller gains
    K = 9 % Chosen completely arbitrarily, can't find in paper
    deltaT = 15 % ms, time delay, from Fig 5 in Cheng paper, chosen somewhat arbitrarily from allowable
    tauC = 40 %ms, controller time constant from Fig5 as above
    Kp = K*tauC
    Ki = K
    
    %% P(s) parameters
    I = 2119 % moment of inertia for magnificent hummingbird pitch
    tauP = 69.7 %I/b, the open-loop time constant in ms
    
    sim("BirdModel.slx")
    plot(ans)
end
