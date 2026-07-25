function get_RRR_G_sfun(block)
    setup(block);
end

function setup(block)
    % 1 Input port (Q) and 1 Output port (Gravity Vector G)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
    
    % Input Port (Q: 3x1 joint positions)
    block.InputPort(1).Dimensions        = [3 1]; 
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).DataTypeId        = 0; % double
    
    % Output Port (G: 3x1 Gravity Torque Vector)
    block.OutputPort(1).Dimensions       = [3 1]; 
    block.OutputPort(1).DataTypeId       = 0; % double
    
    block.SampleTimes = [-1 0]; % Inherited
    block.SimStateCompliance = 'DefaultSimState';
    block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
    Q = block.InputPort(1).Data;
    
    % Call your gravity calculation file
    EstTau_G = get_RRR_G(Q); 
    
    block.OutputPort(1).Data = EstTau_G;
end
