# SoMainline Linux repo

Hello!

This is our Linux repo where we push our work! Our branches follow the following naming scheme:

```<owner>/<version>-<name>```

e.g.

```konrad/v6.0-rc3-sm8350```

Branches with owner = `som` (for SoMainline) are expected to be more stable (functionality-wise) and will only be forcepushed if truly necessary.


Branches starting with team member names (especially if they are based on `linux-next`) are *expected* to be forcepushed and can contain experimental/untested/dirty/not-compiling code - if you're an end user, you should probably stay away from these, until the changes get picked into our stable branches.


Feel free to open issues and pull requests if you want to help us!


Unless stated otherwise (for example by having `[DONOTUPSTREAM]` in the commit title or lacking a proper commit message and a `Signed-off-by:` tag), all of our commits are meant to be submitted to the mainline Linux kernel. We try to do that as soon as we're sure beyond reasonable doubt they're functional and meet our code quality standards.


We ask you not to submit our patches on our behalf (unless we agree to, for example in an email or a DM), as the reason we didn't send them ourselves yet is probably that they still need more work.


Please remember that we're doing this free of charge in our spare time. If you enjoy our work, you can support our efforts via GitHub Sponsors by clicking the Sponsors button. Some team members also accept individual donations.
