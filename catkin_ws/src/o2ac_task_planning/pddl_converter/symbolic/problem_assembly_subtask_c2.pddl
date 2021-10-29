(define (problem wrs-assembly-subtask-c2)
  (:domain skill-database)
  (:objects
    shaft end_cap bearing shaft-end_cap shaft-end_cap-bearing - assembly
    a_bot b_bot - robot
  )
  (:init
    ; Robots are free
    (not (robot-carries-an-object a_bot))
    (not (robot-carries-an-object b_bot))

    ; Parts init
    (complete bearing)
    (not (available bearing))
      (incorporated bearing shaft-end_cap-bearing)

    (complete shaft)
    (available shaft)
      (requires-reorientation shaft)
      (not (handover-allowed shaft))

    (complete end_cap)
    (available end_cap)
      (requires-insertion end_cap)
      (requires-reorientation end_cap)
      (not (handover-allowed end_cap))

    (requires-insertion shaft-end_cap)

    ; Assembly init

    (part-of shaft shaft-end_cap)
    (part-of end_cap shaft-end_cap)

    (assemble-order shaft end_cap shaft-end_cap)
    
    (part-of bearing shaft-end_cap-bearing)    
    (part-of shaft-end_cap shaft-end_cap-bearing)    
    
    (assemble-order bearing shaft-end_cap shaft-end_cap-bearing)
        
  )
  (:goal (and
          (complete shaft-end_cap-bearing)
          (complete shaft-end_cap)
         )
  )
  
)