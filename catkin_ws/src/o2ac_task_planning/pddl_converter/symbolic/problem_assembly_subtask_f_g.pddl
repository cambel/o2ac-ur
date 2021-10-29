(define (problem wrs-assembly-subtask-f-g)
  (:domain skill-database)
  (:objects
    base panel_bearing panel_motor sub-assembly-base-panel_bearing sub-assembly-base-panel_motor - assembly
    a_bot b_bot - robot
  )
  (:init
    ; Robots are free
    (not (robot-carries-an-object a_bot))
    (not (robot-carries-an-object b_bot))

    ; Parts init
    (available base)
      (incorporated base sub-assembly-base-panel_bearing)
      (incorporated base sub-assembly-base-panel_motor)

    (available panel_bearing)
      (requires-screws panel_bearing)
      (requires-reorientation panel_bearing)
      (handover-allowed panel_bearing)

    (available panel_motor)
      (requires-screws panel_motor)
      (requires-reorientation panel_motor)
      (handover-allowed panel_motor)

    ; Assembly init

    (part-of base sub-assembly-base-panel_bearing)
    (part-of base sub-assembly-base-panel_motor)

    (part-of panel_bearing sub-assembly-base-panel_bearing)
        
    (part-of panel_motor sub-assembly-base-panel_motor)
    
    (assemble-order base panel_bearing sub-assembly-base-panel_bearing)
    (assemble-order base panel_motor sub-assembly-base-panel_motor)
        
  )
          
  (:goal (and
          (complete sub-assembly-base-panel_bearing)
          (complete sub-assembly-base-panel_motor)
         )
  )
  
)