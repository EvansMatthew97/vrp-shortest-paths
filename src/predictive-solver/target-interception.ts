import { Vecta } from 'vecta';

/**
 * Finds the intercept of a vehicle moving at a constant velocity and
 * a target moving along a straight line also moving at a constant
 * velocity.
 * Treats the maximum distance the vehicle can travel as a circle, and
 * finds  the intercept between that circle and the target.
 * @param line The line segment represented as [[lon, lat], [lon, lat]]
 * @param targetSpeed The speed of the target in degrees/second
 * @param interceptorPosition The position of the interceptor vehicle
 * @param interceptorSpeed The speed in degrees/second of the interceptor vehicle
 * @param timeDelay How long in seconds the target waits before starting to move
 */
export function findTargetLineInterception(line: [[number, number], [number, number]], targetSpeed: number, interceptorPosition: [number, number], interceptorSpeed: number, timeDelay: number = 0) {
  // get bearing of the line, where the x axis is 0 degrees
  const bearing = new Vecta(line[0][0], line[0][1]).angleToRad(new Vecta(line[1][0], line[1][1]));

  // gradient of the line = the gradient the target moves along
  const lineGradientVector = new Vecta(Math.cos(bearing), Math.sin(bearing));

  // calculate the target's movement vector ("gradient" * speed)
  const targetMovementVector = lineGradientVector.mulScalar(targetSpeed); // velocity vector of target

  // target start at t = 0 is equal to the start of the line. However, if this is not
  // the first line segment being tested, we imagine that the line is longer to account
  // for the previous line segments. The line segment is made longer on the start end.
  const targetStartVector = lineGradientVector.invert()
    .mulScalar(timeDelay * targetSpeed);
  const targetStart = new Vecta(line[0][0], line[0][1]).add(targetStartVector);

  // the starting points of the target and vehicle
  const vehicleStart = new Vecta(interceptorPosition[0], interceptorPosition[1]); // starting vehicle position

  const a = targetMovementVector.dotProduct(targetMovementVector) - interceptorSpeed * interceptorSpeed;
  const b = 2 * targetStart.sub(vehicleStart).dotProduct(targetMovementVector);
  const c = targetStart.sub(vehicleStart).dotProduct(targetStart.sub(vehicleStart));

  // solve quadratic equation for a, b, c to find time
  const solutionA = solveQuadratic(a, b, c, 1);
  const solutionB = solveQuadratic(a, b, c, -1);

  // only positive solution is the correct one (time cannot be negative)
  const interceptTime = Math.max(solutionA, solutionB);

  // find the point along the line the target would be at for the found time
  return targetStart.add(targetMovementVector.mulScalar(interceptTime));
}

/**
 * Solves a quadratic equation
 * @param a 
 * @param b 
 * @param c 
 * @param multiplier There are two ways to perform the equation - one with a - and one with a +
 */
function solveQuadratic(a: number, b: number, c: number, multiplier: -1 | 1 = 1) {
  return (-1 * b + multiplier * Math.sqrt(b * b - 4 * a * c)) / (2 * a);
}