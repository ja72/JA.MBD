# JA.MBD

Simple MBD solver in C# for simulating rigid bodies in 3D using momentum in the state vector.

# Notes

To keep accuracy depending on the maximum rotational speed of an object **ω**, the time step must be kept below **δt** < **δθ**/**ω** where **δθ** is the angular resolution desired. I recommend the angular resulution to be < 1 degrees, or the time step to be **δt** < π/(180* **ω**)
