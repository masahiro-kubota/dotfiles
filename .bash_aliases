alias d='cd ~/Documents'

alias ..2='cd ../..'
alias ..3='cd ../../..'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

alias g='git'
alias ga='git add'
alias ga.='git add .'
alias gd='git diff'
alias gs='git status'
alias gp='git push'
alias gb='git branch'
alias gst='git status'
alias gco='git checkout'
alias gf='git fetch'
alias gc='git commit'
alias gcm='git commit -m'
alias gpom='git push origin main'

# dotfiles
alias bashrc='source ~/.bashrc'
alias zshrc='source ~/.zshrc'

# ros
# ros2 run logging_demo logging_demo_main 2>&1 | tslog
alias tslog='while read line; do echo "$(date "+[%Y-%m-%d %H:%M:%S.%3N]") $line"; done'
alias psim='mkdir -p psim_log && ros2 launch autoware_launch planning_simulator.launch.xml 2>&1 | tslog > psim_log/`date +%m_%d_%H_%M_%S`.log'
alias si='source install/setup.bash'
alias rl='ros2 launch'
alias rr='ros2 run'
