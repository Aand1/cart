# bash-callable (for tab-complete) function for loading bookmarks
setpose()
{
    rosrun pr2_pose_saver load.py `rospack find pr2_pose_saver`/saved_poses/$1
}
# bash-callable (for tab-complete) function for saving bookmarks
savepose()
{
    rosrun pr2_pose_saver save.py `rospack find pr2_pose_saver`/saved_poses/$1
}
# For tab completion on pose_saver stuff
_get_poses()
{
    local curw
    COMPREPLY=()
    curw=${COMP_WORDS[COMP_CWORD]}
    poses_dir=`rospack find pr2_pose_saver`/saved_poses
    COMPREPLY=($(compgen -W "`ls $poses_dir`" -- $curw))
    return 0
}
complete -F _get_poses -o filenames setpose
