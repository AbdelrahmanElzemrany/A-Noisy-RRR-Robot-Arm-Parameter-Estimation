function get_RRR_M_sfun(block)
    setup(block);
end

function setup(block)
    % 1 Input port (Q) and 1 Output port (Mass Matrix M)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
    
    % Input Port (Q: 3x1 joint positions)
    block.InputPort(1).Dimensions        = [3 1]; 
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).DataTypeId        = 0; % double
    
    % Output Port (M: 3x3 Inertia Matrix)
    block.OutputPort(1).Dimensions       = [3 3]; 
    block.OutputPort(1).DataTypeId       = 0; % double
    
    block.SampleTimes = [-1 0]; % Inherited
    block.SimStateCompliance = 'DefaultSimState';
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    Q = block.InputPort(1).Data;
    
    % Call your unoptimized 000 character file
    EstTau_M = get_RRR_M(Q); 
    
    block.OutputPort(1).Data = EstTau_M;
end
