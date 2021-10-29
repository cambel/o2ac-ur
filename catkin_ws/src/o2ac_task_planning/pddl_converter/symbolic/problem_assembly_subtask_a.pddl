(define (problem wrs-assembly-subtask-a)
  (:domain skill-database)
  (:objects
    motor panel_motor panel_motor-motor - assembly
    a_bot b_bot - robot
  )
  (:init
    ; Robots are free
    (not (robot-carries-an-object a_bot))
    (not (robot-carries-an-object b_bot))

    ; Parts init
    (complete panel_motor)
    (not (available panel_motor))
    (incorporated panel_motor panel_motor-motor)

    (complete motor)
    (available motor)
      (requires-reorientation motor)
      (requires-insertion motor)
      (requires-screws motor)

    ; Assembly init

    (part-of motor panel_motor-motor)
    (part-of panel_motor panel_motor-motor)

    (assemble-order panel_motor motor panel_motor-motor)
    
        
  )
  (:goal (complete panel_motor-motor))
  
)