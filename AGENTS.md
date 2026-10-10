Hundreds of hours of work on this project has revealed a surprising truth: 

# Error messages shown in the UI are more likely to be bugs in the algorithm than user errors. 

Put another way, by week 5 of development, no beta tester or developer had yet encountered a genuine error message. ***Every single one was a bug***.

Because of this, when writing code for the UI that can fail for algorithmic reasons:
- Do not EVER show an error dialog when there's a better solution.
  - "Error: Pipes are 3mm out of alignment and because this is a draft finalisation I should move them, but I thought I'd annoy you instead."
  - "Error: Can't connect this chain to this part because that's what you want, and it's clearly possible, but that would be too easy."
  - "Error: Pipes are perfectly positioned because they're all snapped in and validation passes, but it'd be fun to say they're 44mm too far into a connector even if every one rendered is perfect."
- There is usually only a very rare corner case in which it's acceptable to show an error during connection, validation, or draft finalisation, and that's provably impossible topology like MC Escher art. You can safely assume it's not until we start building actual tests for those cases.
- Usually the best solution instead of showing an error is to do the action to user requested and ensure that subtle complications or pedantry are handled with good design.

When a bug is identified in an algorithm that has fuzzing:
- Add a regression test for it.
- Ensure that all the behaviour that triggered the bug makes it way into the fuzz matrix.
- Ensure that both the fuzz tester and regression test find it - a repeatedly passing fuzz test with any known bug going undiscovered is a major issue, and should result in an increase in default fuzzing time at the bare minimum. 

After a major change or several smaller ones, run the fuzz tests. Even if they run for hours it's worth it. Debug and investigate any random failures or incorrect error dialogs.

